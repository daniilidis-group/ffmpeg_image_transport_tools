# ffmpeg_image_transport_tools

This repo contains tools that work in conjunction with the
[ffmpeg_image_transport](https://github.com/daniilidis-group/ffmpeg_image_transport)

## decoding bags

So let's say you have collected video data using the ffmpeg image
transport, and it's sitting in a ROS bag. You can decode it with ``decode_bag``!
First have a look at the ``decode_bag_single.launch`` script to make sure the
image topic is correct etc.


	roslaunch ffmpeg_image_transport_tools decode_bag_single.launch bag:=/archive/tmp/test.bag out_file_dir:=/data/tmp

This will yield a raw h256 file. Now use ``ffmpeg`` to put it into an ``mp4`` container:

    ffmpeg -framerate 40 -i video_full.h265 -c copy video.mp4

You can stack several video streams next to each other by using th ``decode_bag.launch`` script.
For image tiling, please see instructions in the script.

    roslaunch ffmpeg_image_transport_tools decode_bag.launch bag:=/archive/tmp/test.bag out_file_dir:=/data/tmp

If you just want the raw video stream, you can use the ``split_bag`` node,
which pulls the video(s) out of the bag and splits them into one stream for each camera:

    roslaunch ffmpeg_image_transport_tools split_bag.launch bag:=your_bag_name.bag out_file_base:=/your_director/video_

This should produce files ``video_*.h265``, one for each camera, which again you can turn into
a mp4 file using ffmpeg.

## License

This software is issued under the Apache License Version 2.0.
