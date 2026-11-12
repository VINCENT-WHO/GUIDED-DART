#pragma once
void camera_command(const char *command);
int camera_video_prepare(unsigned *width,unsigned *height);
int camera_video_frame(const unsigned char **data,unsigned *bytes);
void camera_record_video(unsigned token);
