import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

width = 640
height = 480

pc = rs.pointcloud()
points = rs.points()

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
# profile = pipeline.get_active_profile()
# sensor = profile.get_device().query_sensors()[0]  # 0 for depth sensor, 1 for camera sensor
# sensor.set_option(rs.option.max_distance, 0)
# sensor.set_option(rs.option.enable_max_usable_range, 0)
colorizer = rs.colorizer()
colorizer.set_option(rs.option.visual_preset, 2)
print("Depth Scale is: ", depth_scale)
depth_sensor.set_option(rs.option.laser_power, 100)
laser_pwr = depth_sensor.get_option(rs.option.laser_power)
print("laser power = ", laser_pwr)

laser_range = depth_sensor.get_option_range(rs.option.laser_power)
print("laser power range = " , laser_range.min , "~", laser_range.max)
try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        #if not depth_frame or not color_frame:
            #continue

        # convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        print("The size of depth_image is:", depth_image.shape)
        color_image = np.asanyarray(color_frame.get_data())
        print("The size of color_image is:", color_image.shape)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth = depth_image[320,240].astype(float)*depth_scale

        cv2.imshow('rgb', color_image)
        cv2.imshow('depth', depth_colormap)
        print(f'Depth: {depth} m')

        # Create save_to_ply object
        ply = rs.save_to_ply("1.ply")

        # Set options to the desired values
        # In this example we'll generate a textual PLY with normals (mesh is already created by default)
        ply.set_option(rs.save_to_ply.option_ply_binary, False)
        ply.set_option(rs.save_to_ply.option_ply_normals, True)

        print("Saving to 1.ply...")
        # Apply the processing block to the frameset which contains the depth frame and the texture
        ply.process(colorized)

        if cv2.waitKey(1) == ord("q"):
            break
finally:
    pipeline.stop()

