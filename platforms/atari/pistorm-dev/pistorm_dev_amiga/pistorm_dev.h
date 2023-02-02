// SPDX-License-Identifier: MIT

unsigned int pi_find_pistorm(void);

unsigned short pi_get_hw_rev(void);
unsigned short pi_get_sw_rev(void);
unsigned short pi_get_net_status(void);
unsigned short pi_get_rtg_status(void);
unsigned short pi_get_piscsi_status(void);

void pi_enable_rtg(unsigned short val);
void pi_enable_net(unsigned short val);
void pi_enable_piscsi(unsigned short val);

unsigned short pi_get_temperature(void);
unsigned short pi_get_rtg_scale_mode(void);
unsigned short pi_get_rtg_scale_filter(void);

void pi_reset_amiga(unsigned short reset_code);
unsigned short pi_handle_config(unsigned char cmd, char *str);

void pi_set_feature_status(unsigned short cmd, unsigned char value);

unsigned short pi_piscsi_map_drive(char *filename, unsigned char index);
unsigned short pi_piscsi_unmap_drive(unsigned char index);
unsigned short pi_piscsi_insert_media(char *filename, unsigned char index);
unsigned short pi_piscsi_eject_media(unsigned char index);

unsigned short pi_get_filesize(char *filename, unsigned int *file_size);
unsigned short pi_transfer_file(char *filename, unsigned char *dest_ptr);
unsigned short pi_memcpy(unsigned char *dst, unsigned char *src, unsigned int size);
unsigned short pi_memset(unsigned char *dst, unsigned char val, unsigned int size);
void pi_copyrect(unsigned char *dst, unsigned char *src, unsigned short src_pitch, unsigned short dst_pitch, unsigned short w, unsigned short h);
void pi_copyrect_ex(unsigned char *dst, unsigned char *src, unsigned short src_pitch, unsigned short dst_pitch, unsigned short src_x, unsigned short src_y, unsigned short dst_x, unsigned short dst_y, unsigned short w, unsigned short h);
void pi_copyrect_ex_mask(unsigned char *dst, unsigned char *src, unsigned short src_pitch, unsigned short dst_pitch, unsigned short src_x, unsigned short src_y, unsigned short dst_x, unsigned short dst_y, unsigned short w, unsigned short h, unsigned char mask_color);
void pi_fill_rect(unsigned char *dst, unsigned short pitch, unsigned short x, unsigned short y, unsigned short w, unsigned short h, unsigned int color);
unsigned int pi_get_fb(void);
void pi_set_rtg_scale_mode(unsigned short scale_mode);
void pi_set_rtg_scale_rect(unsigned short scale_mode, signed short x1, signed short y1, signed short x2, signed short y2);
void pi_set_rtg_scale_filter(unsigned short scale_filter);

unsigned short pi_load_config(char *filename);
void pi_reload_config(void);
void pi_load_default_config(void);

unsigned short pi_remap_kickrom(char *filename);
unsigned short pi_remap_extrom(char *filename);

unsigned short pi_shutdown_pi(unsigned short shutdown_code);
unsigned short pi_confirm_shutdown(unsigned short shutdown_code);

void pi_show_clut_mouse_cursor(unsigned char show);
void pi_set_clut_mouse_cursor(short hot_x, short hot_y, unsigned short w, unsigned short h, const void *bmp, unsigned int key_color, unsigned char *pal_pointer);
