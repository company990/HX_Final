HX_Final 电脑工具

本目录的工具在电脑上运行，不加入A/B板TD工程，不烧录到FPGA。

PC_SENDER/Internet_transmit.exe：现有NET26 PACED发送程序。
PC_SENDER/Internet_transmit.cpp：对应C++源码。
PC_SENDER/Internet_transmit_diag.exe及同名cpp：协议兼容的诊断版，增加速率/重传日志及无效ACK时的超时检查。
两个程序共用PC_SENDER/tools/ffmpeg；请整体保留此目录。
从程序打印的Media folder读取MP4/WAV；通常为当前用户桌面DATA_FOLDER。
输入编号开始，r刷新，q退出。协议和FPGA固件均未在本次打包中修改。
源码可用MinGW-w64编译；例如在PC_SENDER目录执行：
g++ -O2 -std=c++11 -Wall -Wextra -static -static-libgcc -static-libstdc++ Internet_transmit.cpp -lws2_32 -o Internet_transmit.exe

TF_VIDEO_CONVERTER：保留三个Python脚本的相对位置。
需要Python、Pillow和NumPy，可先运行python -m pip install pillow numpy。
在本tools目录打开终端，转换示例：
python ./TF_VIDEO_CONVERTER/convert_tf_video_rle.py "D:/input.mp4" --output "D:/TF_OUTPUT/VIDEO.FVP" --seconds 30 --ffmpeg "./PC_SENDER/tools/ffmpeg/ffmpeg-9.0.2-essentials_build/bin/ffmpeg.exe"
输出FVP4为320x240/20fps、48kHz单声道PCM16，从输入开头取指定时长。
需要指定起点时先用FFmpeg剪辑，再转换；已有FVP3可用convert_fvp_rle.py无损重打包。
只把VIDEO.FVP放入TF卡根目录，自动生成的JSON不需要放卡。
本包不含TF卡素材和以太网演示视频；已有TF卡可继续使用，新素材请单独准备。
电脑负责MP4离线解码，FPGA负责FVP/RLE播放或网络RGB565/PCM接收与输出。
