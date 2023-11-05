import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

class Camera:
    def __init__(self) -> None:
        self.counter = 0
        self.width = 640
        self.height = 480

        self.pc = rs.pointcloud()
        self.points = rs.points()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
        
        self.profile = self.pipeline.start(self.config)

        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        # profile = pipeline.get_active_profile()
        # sensor = profile.get_device().query_sensors()[0]  # 0 for depth sensor, 1 for camera sensor
        # sensor.set_option(rs.option.max_distance, 0)
        # sensor.set_option(rs.option.enable_max_usable_range, 0)
        self.colorizer = rs.colorizer()
        self.colorizer.set_option(rs.option.visual_preset, 2)
        print("Depth Scale is: ", self.depth_scale)
        self.depth_sensor.set_option(rs.option.laser_power, 100)
        self.laser_pwr = self.depth_sensor.get_option(rs.option.laser_power)
        print("laser power = ", self.laser_pwr)

        self.laser_range = self.depth_sensor.get_option_range(rs.option.laser_power)
        print("laser power range = " , self.laser_range.min , "~", self.laser_range.max)

    def get_pic():
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        #print("The size of depth_image is:", depth_image.shape)
        color_image = np.asanyarray(color_frame.get_data())
        #print("The size of color_image is:", color_image.shape)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        RGB_path = "runtime_pic/RGB_"+str(self.counter)+".jpg"
        depth_colormap_path = "runtime_depthpic/depthpic_"str(self.depth_scale)+"_"+str(self.counter)+".jpg"
        depth_map_path = "runtime_depth/depth_"str(self.depth_scale)+"_"+str(self.counter)+".txt"

        cv2.imwrite(RGB_path, color_image)
        cv2.imwrite(depth_colormap_path, depth_colormap)
        np.savetxt(depth_map_path, depth_image)
        # depth = depth_image[320,240].astype(float)*depth_scale
        self.counter += 1
        self.counter %= 60
        return (RGB_path, depth_colormap_path, depth_map_path) 


















# def get_frame():
#     global is_on
#     global pipeline
#     if not is_on:
#         setup_camera()

#     frames = pipeline.wait_for_frames()
#     depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()

#     # convert images to numpy arrays
#     depth_image = np.asanyarray(depth_frame.get_data())
#     #print("The size of depth_image is:", depth_image.shape)
#     color_image = np.asanyarray(color_frame.get_data())
#     #print("The size of color_image is:", color_image.shape)
#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#     # depth = depth_image[320,240].astype(float)*depth_scale

#     return (color_image, depth_image, depth_colormap)

# def get_pic():
#     global is_on
#     global pipeline
#     if not is_on:
#         setup_camera()

#     frames = pipeline.wait_for_frames()
#     depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()

#     # convert images to numpy arrays
#     depth_image = np.asanyarray(depth_frame.get_data())
#     #print("The size of depth_image is:", depth_image.shape)
#     color_image = np.asanyarray(color_frame.get_data())
#     #print("The size of color_image is:", color_image.shape)
#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
#     cv2.imwrite("runtime_pic/")
#     # depth = depth_image[320,240].astype(float)*depth_scale

#     return (color_image, depth_image, depth_colormap)    

# def setup_camera():
#     global pipeline
#     global depth_scale
#     global is_on
#     #TODO: CHange here
    

# if __name__ == "__main__":
#     count = 0
#     try:
#         while True:
#             color_image, depth_image, depth_colormap = get_frame()
#             depth = depth_image[320,240].astype(float)*depth_scale
            
#             cv2.imshow('rgb', color_image)
#             cv2.imshow('depth', depth_colormap)
#             count += 1
#             if (count % 100 == 0):
#                 cv2.imwrite("test/color_image"+str(count//100)+ ".jpg", color_image)
#                 cv2.imwrite("test/depth"+ str(depth_scale) + "_image"+str(count//100)+ ".jpg", depth_colormap)
#                 np.savetxt("test/test_depth" + str(depth_scale) +"_image" + str(count//100)+".txt", depth_image)
#                 print("Stored!")
#                 print(f'Depth: {depth} m')
#                 print(f'Depth_scale:{depth_scale} m')
#             if cv2.waitKey(1) == ord("q"):
#                break  
#     except KeyboardInterrupt:
#         pipeline.stop()
#         exit()
# try:
#     while True:
#         frames = pipeline.wait_for_frames()
#         depth_frame = frames.get_depth_frame()
#         color_frame = frames.get_color_frame()
#         #if not depth_frame or not color_frame:
#             #continue

#         # convert images to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         print("The size of depth_image is:", depth_image.shape)
#         color_image = np.asanyarray(color_frame.get_data())
#         print("The size of color_image is:", color_image.shape)
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#         depth = depth_image[320,240].astype(float)*depth_scale

#         cv2.imshow('rgb', color_image)
#         cv2.imshow('depth', depth_colormap)
#         print(f'Depth: {depth} m')

#         # Create save_to_ply object
#         ply = rs.save_to_ply("1.ply")

#         # Set options to the desired values
#         # In this example we'll generate a textual PLY with normals (mesh is already created by default)
#         ply.set_option(rs.save_to_ply.option_ply_binary, False)
#         ply.set_option(rs.save_to_ply.option_ply_normals, True)

#         print("Saving to 1.ply...")
#         # Apply the processing block to the frameset which contains the depth frame and the texture
#         ply.process(colorized)
#         scene = a3d.Scene.from_file("1.ply")
#         scene.save("Output.glb")

#         if cv2.waitKey(1) == ord("q"):
#             break
# finally:
#     pipeline.stop()

