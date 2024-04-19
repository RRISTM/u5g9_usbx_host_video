# STM32U5G HOST with USBX for video class standalone(no RTOS)

video calss is handled in `app_usbx_host.c`

In `ux_host_event_callback` is handled invertion or remove of the device

In insertion the video calss resolution is set and refresh rate and class is started. 

The function `video_read` is perioadically from main. To start new reception with `ux_host_class_video_read`

The received data will call callack `video_transfer_complete`

Currently the reception is in uncopressed format YUV422. So it must be converted to RGB. 

Currently set parameters:
resolution 320x240
refresh 200ms (5fps)
uncompressed (YUV422)

Currently only one camera was tested. Most of the time probably the yUV conversion is take much of the time. 