//#define DISASSEMBLE
// SPDX-License-Identifier: MIT

#include "platforms/platforms.h"
#include "platforms/atari/IDE.h"
#include "platforms/atari/idedriver.h"
#include "platforms/atari/atari-registers.h"
#include "platforms/atari/pistorm-dev/pistorm-dev-enums.h"
#include "gpio/ps_protocol.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sched.h>
//#ifdef RTG
#include <pthread.h>
//#endif
#include "m68kops.h"
#include <stdbool.h>
#include "platforms/atari/et4000.h"



#define DEBUGPRINT 1
#if DEBUGPRINT
#define DEBUG_PRINTF(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...) ;
#endif

void *ide_task ( void* );
void *misc_task ( void* vptr );

extern char *get_pistorm_cfg_filename();
extern void set_pistorm_cfg_filename (char *);

int FPU68020_SELECTED;
extern uint8_t IDE_IDE_enabled;
extern volatile unsigned int *gpio;
extern uint8_t fc;

uint8_t emulator_exiting = 0;
volatile uint32_t old_level;

volatile uint32_t last_irq = 8;
volatile uint32_t last_last_irq = 8;

extern volatile int g_irq;
extern volatile int g_buserr;

uint8_t load_new_config = 0;

int mem_fd;
int mem_fd_gpclk;
volatile int irq;
volatile int cpu_emulation_running = 0;
volatile int passthrough = 0;

//#ifdef RTG
extern void rtg ( int, uint32_t, uint32_t );
//extern void rtg_raylib ( int, uint32_t, uint32_t );
extern volatile uint32_t RTG_ATARI_SCREEN_RAM;
extern volatile uint32_t RTG_VSYNC;
//extern int RTG_enabled;
//extern volatile uint32_t RTG_VRAM_BASE;
//extern volatile uint32_t RTG_VRAM_SIZE;
//#endif

extern int RTG_enabled;
volatile uint32_t RTG_VRAM_BASE;
volatile uint32_t RTG_VRAM_SIZE;

FILE *console = NULL;

#define MUSASHI_HAX

#ifdef MUSASHI_HAX
#include "m68kcpu.h"
extern m68ki_cpu_core m68ki_cpu;
extern int m68ki_initial_cycles;
extern int m68ki_remaining_cycles;

#define M68K_END_TIMESLICE 	m68ki_initial_cycles = GET_CYCLES(); \
	SET_CYCLES(0);
#else
#define M68K_SET_IRQ m68k_set_irq
#define M68K_END_TIMESLICE m68k_end_timeslice()
#endif

// Configurable emulator options
unsigned int cpu_type = M68K_CPU_TYPE_68000;
unsigned int loop_cycles = 20, irq_status = 0;
struct emulator_config *cfg = NULL;

volatile uint32_t do_reset=0;
volatile uint32_t gotIntLevel;
volatile uint32_t ipl;

//void call_berr(uint16_t status, uint32_t address, uint mode);
static inline void m68k_execute_bef(m68ki_cpu_core *, int);

volatile uint32_t g_vector;

extern volatile unsigned int *gpio;
uint16_t irq_delay = 0;
#define TIMEOUT 10000000ul; // arbitary value, so long as it's big enough

#define ORIGINAL


#if (0)
void *ipl_task ( void *args ) 
{
  uint16_t old_irq = 0;
  uint32_t value;
  int watchdog;
  uint32_t poweredOff;;

  irq         = 0;
  iack        = 1;
  poweredOff  = 0;
  watchdog    = TIMEOUT;

  usleep (1000000); // 1s delay to sync threads

  while ( cpu_emulation_running ) 
  {   
    while ( ( ( value = *(gpio + 13) ) & (1 << PIN_TXN_IN_PROGRESS) ) && watchdog ) 
    {
      //NOP
      watchdog--;
    }

    if ( watchdog )
    {
      if ( ! ( value & (1 << PIN_IPL_ZERO) ) )
      {
      // if (!iack && !irq)
        if (!irq)
        {
          //if (!iack)
          //  printf ( "missed int %d\n", last_irq );

          irq = 1; 
        }
      }

      else 
      {
        if (irq)
          irq = 0;
      }  
    }

    if ( !watchdog && !poweredOff )
    {
      DEBUG_PRINTF ( "[ATARI] Powered OFF / Unresponsive\n" );
      poweredOff  = 1;
    }

    else if ( poweredOff && watchdog )
    {
      DEBUG_PRINTF ( "[ATARI] Powered ON\n" );
      poweredOff  = 0;
      do_reset    = 1;
    }

    watchdog      = TIMEOUT;
  }

  return args;
}
#endif



//#ifdef RTG
void cpu2 ( void )
{
  cpu_set_t cpuset;
	CPU_ZERO ( &cpuset );
	CPU_SET ( 2, &cpuset );
	sched_setaffinity ( 0, sizeof (cpu_set_t), &cpuset );
}
//#endif

void cpu3 ( void )
{
  cpu_set_t cpuset;
	CPU_ZERO ( &cpuset );
	CPU_SET ( 3, &cpuset );
	sched_setaffinity ( 0, sizeof (cpu_set_t), &cpuset );
}



extern uint m68ki_read_imm16_addr_slowpath ( m68ki_cpu_core *state, uint32_t pc );

static inline void m68k_execute_bef ( m68ki_cpu_core *state, int num_cycles )
{
	/* eat up any reset cycles */
  /*
	if (RESET_CYCLES) 
  {
	    int rc = RESET_CYCLES;
	    RESET_CYCLES = 0;
	    num_cycles -= rc;
	    if (num_cycles <= 0)
		return;
	}
  */
	/* Set our pool of clock cycles available */
	SET_CYCLES (num_cycles);
	m68ki_initial_cycles = num_cycles;

	/* Make sure we're not stopped */
	if ( !CPU_STOPPED )
	{
		/* Main loop.  Keep going until we run out of clock cycles */
execute:
    m68ki_use_data_space ();

    REG_PPC = REG_PC;
    //REG_IR = m68ki_read_imm_16 ( state );
    REG_IR = m68ki_read_imm16_addr_slowpath ( state, REG_PC );

    m68ki_instruction_jump_table [REG_IR] (state);

    if ( g_buserr ) 
      m68ki_exception_bus_error ( state ); 

    USE_CYCLES ( CYC_INSTRUCTION [REG_IR] );
  
    //if ( GET_CYCLES () < 1 )
    if ( GET_CYCLES () < 1 || g_irq )
    {
      /* set previous PC to current PC for the next entry into the loop */
		  REG_PPC = REG_PC;
      return;
    }
      
    goto execute;
	}

	else
		SET_CYCLES(0);

	return;
}


void sigint_handler ( int sig_num ) 
{
  cpu_emulation_running = 0;
  
  DEBUG_PRINTF ( "\n[MAIN] Exiting\n" );
  
  if ( mem_fd )
    close ( mem_fd );

  if ( cfg->platform->shutdown ) 
  {
    cfg->platform->shutdown ( cfg );
  }

  while ( !emulator_exiting ) 
  {
    emulator_exiting = 1;
    usleep ( 0 );
  }

  exit ( 0 );
}


extern const char *cpu_types[];
extern volatile int RESOLUTION;

#ifdef DISASSEMBLE
char disasm_buf[4096];
#endif

void *cpu_task() 
{
  const struct sched_param priority = {99};
  uint16_t status;
	m68ki_cpu_core *state = &m68ki_cpu;
  state->gpio = gpio;
	m68k_pulse_reset(state);
  g_buserr = 0;	

  
  usleep ( 1000000 );  

  //cpu3 (); // anchor task to cpu3 
  //sched_setscheduler ( 0, SCHED_FIFO, &priority );
  //system ( "echo -1 >/proc/sys/kernel/sched_rt_runtime_us" );

  while ( !cpu_emulation_running )
    ;

run:
  m68k_execute_bef ( state, loop_cycles );

  //if ( !RTG_VSYNC )
  {
    if ( g_irq )
    {
      status = ps_read_status_reg ();
      last_irq = status >> 13;

      if ( status & 0x2 ) 
      {
        M68K_END_TIMESLICE;

        DEBUG_PRINTF ( "[CPU] Emulation reset\n");

        usleep ( 1000000 ); 

        m68k_pulse_reset ( state );

        RESOLUTION = 0;
      }

      else
      {
        /* ET4000 driver uses this */
        //if ( last_irq == 4 )
        //  RTG_VSYNC = 1;

        //else
        //  RTG_VSYNC = 0;

        if ( last_irq != 0 && last_irq != last_last_irq ) 
        {
          m68k_set_irq ( last_irq );
          last_last_irq = last_irq;
        }
        
        m68ki_exception_interrupt ( state, CPU_INT_LEVEL >> 8 );
      }

      //m68ki_exception_interrupt ( state, CPU_INT_LEVEL >> 8 );
    }

    //else if ( !g_irq && last_last_irq != 0 ) 
    else if ( last_last_irq != 0 ) 
    {
      m68k_set_irq ( 0 );
      last_last_irq = 0;
    }
  }

  if ( !cpu_emulation_running )
  {
    DEBUG_PRINTF ("[CPU] End of CPU thread\n");

    return (void *)NULL;
  }
  goto run;
}


extern void rtgInit ( void );
extern void *rtgRender ( void* );

int main ( int argc, char *argv[] ) 
{
  const struct sched_param priority = {99};
  int g;
#ifdef RTG
  int err;
  pthread_t rtg_tid, cpu_tid;
#endif
  RTG_enabled = 0;
  FPU68020_SELECTED = 0;

  // Some command line switch stuffles
  for ( g = 1; g < argc; g++ ) 
  {
    if ( strcmp ( argv[g], "--config-file") == 0 || strcmp(argv[g], "--config" ) == 0 ) 
    {
      if (g + 1 >= argc) 
      {
        DEBUG_PRINTF ( "%s switch found, but no config filename specified.\n", argv[g] );
      } 
      
      else 
      {
        g++;
        FILE *chk = fopen ( argv[g], "rb" );

        if ( chk == NULL ) 
        {
          DEBUG_PRINTF ( "Config file %s does not exist, please check that you've specified the path correctly.\n", argv[g] );
        } 
        
        else 
        {
          fclose ( chk );
          load_new_config = 1;
          set_pistorm_cfg_filename ( argv[g] );
        }
      }
    }
  }

  if ( load_new_config ) 
  {
    uint8_t config_action = load_new_config - 1;
    load_new_config = 0;
    if (cfg) {
      free_config_file(cfg);
      free(cfg);
      cfg = NULL;
    }

    switch(config_action) 
    {
      case PICFG_LOAD:
      case PICFG_RELOAD:
        cfg = load_config_file ( get_pistorm_cfg_filename () );
        break;
      //case PICFG_DEFAULT:
      //  cfg = load_config_file("default.cfg");
      //  break;
    }
  }

  if (!cfg) 
  {
    DEBUG_PRINTF ("No config file specified\n");
    
    return 1;
  }

  if (cfg) 
  {
    if (cfg->cpu_type) 
      cpu_type = cfg->cpu_type;

    if (cfg->loop_cycles) 
      loop_cycles = cfg->loop_cycles;
    
    else if ( loop_cycles == 0 )
      loop_cycles = 12;

    if (!cfg->platform)
    {
      cfg->platform = make_platform_config ( "atari", "st" );
      printf ( "[CFG] Plaform not specified - using atari st\n" );
    }

    cfg->platform->platform_initial_setup ( cfg );
  }

  signal ( SIGINT, sigint_handler );

  mlockall ( MCL_CURRENT );  // lock in memory to keep us from paging out

  

  ps_setup_protocol ();
  ps_reset_state_machine ();
  ps_pulse_reset ();
  usleep (1500);

  m68k_init ();
	m68k_set_cpu_type ( &m68ki_cpu, cpu_type );
  m68k_set_int_ack_callback ( &cpu_irq_ack );
  cpu_pulse_reset ();

  fc = 6;
  g_buserr = 0;

#ifdef MISC_TASK
  // create miscellaneous task

  err = pthread_create ( &misc_tid, NULL, &misc_task, NULL );

  if ( err != 0 )
    DEBUG_PRINTF ( "[ERROR] Cannot create MISCELLANEOUS thread: [%s]", strerror (err) );

  else 
  {
    pthread_setname_np ( misc_tid, "pistorm: misc" );
    DEBUG_PRINTF ( "[MAIN] MISCELLANEOUS thread created successfully\n" );
  }
#endif

#if (0)
#ifdef RTG
  err = pthread_create ( &cpu_tid, NULL, &cpu_task, NULL );

  if ( err != 0 )
    printf ( "[ERROR] Cannot create CPU thread: [%s]", strerror ( err ) );

  else 
  {
    pthread_setname_np ( cpu_tid, "pistorm: cpu" );
    printf ( "[MAIN] CPU thread created successfully\n" );
  }

  // create rtg task
  if (  RTG_enabled )
  {
#ifdef RAYLIB
    int ix = get_named_mapped_item ( cfg, "RTG" );
    if ( cfg->map_data [ix] ) // cryptodad - this will need work - can't rely on it being map[3]
    {
      RTG_VRAM_BASE = cfg->map_offset [ix];
      RTG_VRAM_SIZE = cfg->map_size [ix];
      printf ( "[MAIN] RTG_VRAM_BASE 0x%X, RTG_VRAM_SIZE 0x%X\n", RTG_VRAM_BASE, RTG_VRAM_SIZE );
    }
#endif
    
      err = pthread_create ( &rtg_tid, NULL, &rtgRender, NULL );

      if ( err != 0 )
        DEBUG_PRINTF ( "[ERROR] Cannot create RTG thread: [%s]", strerror (err) );

      else 
      {
        pthread_setname_np ( rtg_tid, "pistorm: rtg" );
        DEBUG_PRINTF ( "[MAIN] RTG thread created successfully\n" );
      }
//#ifdef RAYLIB
    //}
//#endif
  }
#endif
#else

  InitIDE ();
  #ifdef RTG
  rtgInit ();
  #endif
  if ( RTG_enabled )
    et4000Init ();

  int err;
  pthread_t rtg_tid, cpu_tid;

  err = pthread_create ( &cpu_tid, NULL, &cpu_task, NULL );

  if ( err != 0 )
    printf ( "[ERROR] Cannot create CPU thread: [%s]", strerror ( err ) );

  else 
  {
    pthread_setname_np ( cpu_tid, "pistorm: cpu" );
    printf ( "[MAIN] CPU thread created successfully\n" );
  }

  if ( RTG_enabled )
  {
    err = pthread_create ( &rtg_tid, NULL, &rtgRender, NULL );

    if ( err != 0 )
      DEBUG_PRINTF ( "[ERROR] Cannot create RTG thread: [%s]", strerror (err) );

    else 
    {
      pthread_setname_np ( rtg_tid, "pistorm: rtg" );
      DEBUG_PRINTF ( "[MAIN] RTG thread created successfully\n" );
    }
  }

#endif

  /* cryptodad optimisation - .cfg no mappings */
  if ( cfg->mapped_high == 0 && cfg-> mapped_low == 0 )
    passthrough = 1;
  
  else
    passthrough = 0;

  cpu_emulation_running = 1;

  DEBUG_PRINTF ( "[MAIN] Emulation Running [%s%s]\n", cpu_types [cpu_type - 1], (cpu_type == M68K_CPU_TYPE_68020 && FPU68020_SELECTED) ? " + FPU" : "" );

  if ( passthrough )
    DEBUG_PRINTF ( "[MAIN] %s Native Performance\n", cpu_types [cpu_type - 1] );

  if ( RTG_enabled )
    DEBUG_PRINTF ( "[MAIN] Press ESC to terminate\n" );

  else
    DEBUG_PRINTF ( "[MAIN] Press CTRL-C to terminate\n" );

  DEBUG_PRINTF ( "\n" );

  //sched_setscheduler ( 0, SCHED_FIFO, &priority );
  //system ( "echo -1 >/proc/sys/kernel/sched_rt_runtime_us" );
  //cpu3 (); // anchor main task to cpu3 


#if (0)
#ifndef RAYLIB
  cpu_task ();
#else
  if (  RTG_enabled )
    pthread_join ( rtg_tid, NULL );

  else
    pthread_join ( cpu_tid, NULL );
#endif

#else
  if ( RTG_enabled )
    pthread_join ( rtg_tid, NULL );

  else
    pthread_join ( cpu_tid, NULL );
#endif

  DEBUG_PRINTF ("[MAIN] Emulation Ended\n");

  return 0;
}


void cpu_pulse_reset ( void ) 
{
  ps_pulse_reset ();
}



static uint32_t target = 0;
static uint32_t platform_res, rres;
unsigned int garbage = 0;


/* return 24 bit address */
static inline uint32_t check_ff_st( uint32_t add ) 
{
	if ( ( add & 0xFF000000 ) == 0xFF000000 ) 
    add &= 0x00FFFFFF;

	return add;
}




/* levels 2 and 4 are video syncs, so thousands are coming in */
uint16_t cpu_irq_ack ( int level ) 
{
  static uint32_t ack;
  static uint8_t vec;

  fc  = 0x7; // CPU interrupt acknowledge
  ack = 0x00fffff0 | (level << 1);
  vec = ps_read_16 ( ack );
  
  if ( level == 2 || level == 4 ) // autovectors
  	return 24 + level;
  
  return vec;
}




static inline int32_t platform_read_check ( uint8_t type, uint32_t addr, uint32_t *res ) 
{
  if ( IDE_IDE_enabled && (addr >= (0xFF000000 | IDEBASE) && addr < (0xFF000000 | IDETOP) ) )
    addr &= 0x00ffffff;
   // printf ( "IDE read\n" );
  
  //if ( addr >= 0x00ff8a00 && addr < 0x00ff8a3e )
  //{
  //  printf ( "Blitter: read 0x%X\n", addr );

    //*res = 0;
    //return 1;
  //}

  if ( ( addr >= cfg->mapped_low && addr < cfg->mapped_high ) )
  {
    if ( handle_mapped_read ( cfg, addr, &target, type ) != -1 ) 
    {
      *res = target;
      
      return 1;
    }
  }

  //*res = 0;

  return 0;
}


unsigned int m68k_read_memory_8 ( uint32_t address ) 
{
  if ( platform_read_check ( OP_TYPE_BYTE, address, &platform_res ) ) 
  {
    return platform_res;
  }

  //address = check_ff_st ( address );
  if ( ( address & 0xFF000000 ) == 0xFF000000 ) 
    address &= 0x00FFFFFF;

  if ( address & 0xFF000000 )
    return 0;

  return ps_read_8 ( address );  
}


unsigned int m68k_read_memory_16 ( uint32_t address ) 
{
  if ( platform_read_check ( OP_TYPE_WORD, address, &platform_res ) ) 
  {
    return platform_res;
  }

  if ( ( address & 0xFF000000 ) == 0xFF000000 ) 
    address &= 0x00FFFFFF;

  if ( address & 0xFF000000 )
    return 0;

  return ps_read_16 ( address );
}


unsigned int m68k_read_memory_32 ( uint32_t address ) 
{
  if (platform_read_check ( OP_TYPE_LONGWORD, address, &platform_res ) ) 
  {
    return platform_res;
  }

  if ( ( address & 0xFF000000 ) == 0xFF000000 ) 
    address &= 0x00FFFFFF;

  if ( address & 0xFF000000 )
    return 0;

  return ps_read_32 ( address );
}




#ifdef RTG
#include "platforms/atari/atari-registers.h"

/* convert ST xRRR xGGG xBBB to RGB565 */
//#define toRGB565(d) ( (uint16_t) ( (d & 0x0f00) << 3 | (d & 0x00f0) << 2 | (d & 0x000f) ) )

/* convert STe RRRR GGGG BBBB to RGB565 */
#define toRGB565(d) ( (uint16_t) ( (d & 0x0f00) << 4 | (d & 0x00f0) << 3 | (d & 0x000f) << 1 ) )

#define SYS_VARS     0x00000420
#define _vbclock     0x00000462
#define _frlock      0x00000466
#define SYS_VARS_TOP 0x000005b4
#define PALETTE_REGS 0xffff8240

extern volatile uint16_t RTG_PAL_MODE;
extern volatile uint8_t RTG_RES;
extern volatile int RTGresChanged;
extern volatile uint16_t RTG_PALETTE_REG [16];

//#ifdef RAYLIB
//extern volatile int vramLock;
//#endif

#endif

static inline int32_t platform_write_check ( uint8_t type, uint32_t addr, uint32_t val ) 
{
#ifdef RTG
  if ( RTG_enabled && !RTG_VRAM_BASE )
  {
    /* ATARI System Variables - do before anything else */
    if ( addr >= SYS_VARS && addr < SYS_VARS_TOP )
    {
        /* check palmode word */
        if ( addr == 0x448 )
        {
            RTG_PAL_MODE = val;
        }

        /* check sshiftmd word */
        else if ( addr == 0x44c )
        {
            /* has resolution changed? */
            if ( RTG_RES != val )
            {
                RTG_RES = val;
                RTGresChanged = 1;
            }
        }

        /* check v_bas_ad long */
        else if ( addr == 0x44e )
        {
            RTG_ATARI_SCREEN_RAM = (uint32_t)val;
        }

        //else if ( addr == _vbclock )
        //else if ( addr == 0x70 )
        //{
        //  RTG_VSYNC = 1;
        //}
    }

    /* Palatte Registers - 16 x 16 bit words */
    else if ( addr >= PALETTE_REGS && addr < PALETTE_REGS + 0x20 )
    {
      //printf ( "palette change - type = %d REG %d = 0x%X to RGB565 0x%X\n", type, (addr - PALETTE_REGS) >> 1, (uint16_t)val, toRGB565 ( (uint16_t)val)  );
      if ( type == OP_TYPE_WORD )
        RTG_PALETTE_REG [ (addr - PALETTE_REGS) >> 1 ] = toRGB565 ( (uint16_t)val );
      
      else if ( type == OP_TYPE_LONGWORD )
      {
        RTG_PALETTE_REG [ (addr - PALETTE_REGS) >> 1 ] = toRGB565 ( (uint16_t)(val >> 16) );
        RTG_PALETTE_REG [ ((addr - PALETTE_REGS) >> 1) + 1 ] = toRGB565 ( (uint16_t)(val) );
      }
    }

    else if ( addr >= RTG_ATARI_SCREEN_RAM && addr < (RTG_ATARI_SCREEN_RAM + 0x8000) )
    {
      rtg ( type, addr, val );

      return 0;
    }
  }
//#ifdef RAYLIB
//  else if ( RTG_enabled && RTG_VRAM_BASE )
//  {
//    if ( addr >= RTG_VRAM_BASE && addr < RTG_VRAM_BASE | RTG_VRAM_SIZE )
//    {
//      vramLock = 1;
//    }
//  }
//#endif

#endif 

  if ( IDE_IDE_enabled && (addr >= (0xFF000000 | IDEBASE) && addr < (0xFF000000 | IDETOP) ) )
    addr &= 0x00ffffff;

  if ( ( addr >= cfg->mapped_low && addr < cfg->mapped_high ) ) 
  {
    if ( handle_mapped_write ( cfg, addr, val, type ) != -1 )
    {
//#ifdef RAYLIB
//      vramLock = 0;
//#endif
      return 1;
    }
  }

  return 0;
}



void m68k_write_memory_8 ( uint32_t address, unsigned int value ) 
{
  if ( platform_write_check ( OP_TYPE_BYTE, address, value ) )
    return;
   
  if ( ( address & 0xFF000000 ) == 0xFF000000 ) 
    address &= 0x00FFFFFF;

  if ( address & 0xFF000000 )
    return;

  ps_write_8 ( address, value );
}


void m68k_write_memory_16 ( uint32_t address, unsigned int value ) 
{
  if ( platform_write_check ( OP_TYPE_WORD, address, value ) )
    return;

  if ( ( address & 0xFF000000 ) == 0xFF000000 ) 
    address &= 0x00FFFFFF;

  if ( address & 0xFF000000 )
    return;

  ps_write_16 ( address, value );
}


void m68k_write_memory_32 ( uint32_t address, unsigned int value ) 
{
  if ( platform_write_check ( OP_TYPE_LONGWORD, address, value ) )
    return;

  if ( ( address & 0xFF000000 ) == 0xFF000000 ) 
    address &= 0x00FFFFFF;

  if ( address & 0xFF000000 )
    return;

  ps_write_32 ( address, value );
}


void cpu_set_fc ( unsigned int _fc ) 
{
	fc = _fc;
}



#if (0)
/* cryptodad IDE */

/* ATARI IDE interface expected @ 0x00f00000 */
/* ATARI MFP (68901) @ 0x00fffa00 */
/* MFP active edge register (0x00fffa03) - bit 5 = FDC/HDC interrupt */
/* MFP interrupt enable register B (0x00fffa09) - bit 7 = FDC/HDC */
/* MFP interrupt pending register B (0x00fffa0d) - bit 7 */
void *
ide_task ( void* vptr )
{
  uint32_t ideAddress = 0x00f00000;
  uint32_t mappedAddress = ideAddress - cfg->mapped_low;
  uint16_t *cmd;

  usleep (1000000);

  //cmd = (uint16_t*)lpcmd;

  for ( int n = 0; n < 8; n++ )
  {
    if ( cfg->map_type [n] == MAPTYPE_REGISTER )
    {
      //printf ( "IDE mapping found - index %d\n", n );
      //printf ( "mapped address is 0x%x\n", cfg->mapped_low );
      //printf ( "mapped data is 0x%x\n", cfg->map_data [n] );
      break;
    }
  }

  while (1)
  {
    //if ( *cmd )
    {
     // printf ( "ide_task: cmd add 0x%x\n", *cmd );
      usleep (10000);
    }

    usleep (1);
  }
}



void *
misc_task ( void* vptr )
{
  //uint32_t ideAddress = 0x00f00000;
  //uint32_t vidAddress = 0x00ff8200;
  //uint32_t mappedAddress = ideAddress - cfg->mapped_low;
  //uint16_t *cmd;
  //uint32_t vidbase = 0x0;
  int c;
  int d;
  int doit = 0;

  usleep (1000000);
  //cmd = (uint16_t*)lpcmd;

  ///for ( int n = 0; n < 8; n++ )
  //{
  //  if ( cfg->map_type [n] == MAPTYPE_REGISTER )
  //  {
      //printf ( "IDE mapping found - index %d\n", n );
      //printf ( "mapped address is 0x%x\n", cfg->mapped_low );
      //printf ( "mapped data is 0x%x\n", cfg->map_data [n] );
  //    break;
  //  }
  //}


  while (cpu_emulation_running)
  {
    read ( STDIN_FILENO, &c, 1 );
    
    if ( c == 'v' )
    {
      uint32_t vBase;
      uint8_t vMode;

      doit = 1;

      while ( doit ) 
      {
        if ( ATARI_VID_enabled && canpeek )
        {
          peeking = 1;

          vBase = ps_read_32 ( (t_a32)((uint32_t)0x44e) );
          vMode = ps_read_16 ( (t_a32)((uint32_t)0x44c) ) >> 8;

          peeking = 0;
          doit = 0;
        }
      }

      DEBUG_PRINTF ( "Video Base Address is 0x%08x\n", vBase );
      DEBUG_PRINTF ( "Video Mode is %s\n", vMode == 1 ? "640x200" : vMode == 2 ? "640x400" : "320x200" );
      //DEBUG_PRINTF ( "Video Sync Mode is %s\n", m68k_read_memory_8 ( vidAddress + 0x0a ) & 0x02 ? "60 Hz" : "50 Hz" );
    }

    else if ( c == 'g' )
    {
      uint8_t screenGrab [32000];
      uint32_t n = 0;
      FILE *fp;

      fp = fopen ( "screendump", "w+" );

      doit = 1;

      while ( doit ) 
      {
        if ( ATARI_VID_enabled && canpeek )
        {
          peeking = 1;

          for ( n = 0; n < 32000; n++ )
            screenGrab [n] = ps_read_8 ( (t_a32)((uint32_t)0x3f8000 + n) );

          peeking = 0;
          doit = 0;
        }
      }

      fwrite ( screenGrab, 32000, 1, fp );

      //DEBUG_PRINTF ( "screen grabbed\n" );
    }
  }
}
#endif