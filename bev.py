import cv2
import numpy as np
import math

points = []

class Projection(object):

    def __init__(self, image_path, points):
        """
            :param points: Selected pixels on top view(BEV) image
        """

        if type(image_path) != str:
            self.image = image_path
        else:
            self.image = cv2.imread(image_path)
        self.height, self.width, self.channels = self.image.shape

    def top_to_front(self, theta=0, phi=0, gamma=0, dx=0, dy=0, dz=0, fov=90):
        """
            Project the top view pixels to the front view pixels.
            :return: New pixels on perspective(front) view image
        """

        ### TODO ###
        f = 1
        #print("focal length:", f)
        
        new_pixels = []
        BEV_3D_points = []
        front_3D_points = []
        alpha = 0 
        beta = 0
        gamma = theta * math.pi / 180 # inverse 90 = rotate 90

        tx = 0
        ty = 0.8 #0.8
        tz = 0
        
        Intrinsic_Matrix = np.array([[f, 0, 0, 0],
                                     [0, f, 0, 0],
                                     [0, 0, 1, 0]])
                                     
        # world rotate
        RX = np.array([[1, 0, 0],
                       [0, math.cos(gamma), -math.sin(gamma)],
                       [0, math.sin(gamma), math.cos(gamma)]])
        RY = np.array([[math.cos(beta), 0, math.sin(beta)],
                       [0, 1, 0],
                       [-math.sin(beta), 0, math.cos(beta)]])
        RZ = np.array([[math.cos(alpha), -math.sin(alpha), 0],
                       [math.sin(alpha), math.cos(alpha), 0],
                       [0, 0, 1]])

        tmp = np.dot(RZ, RY)
        RM = np.dot(tmp, RX)
        Extrinsic_Matrix = np.array([[RM[0][0], RM[0][1], RM[0][2], tx],
                                     [RM[1][0], RM[1][1], RM[1][2], ty],
                                     [RM[2][0], RM[2][1], RM[2][2], tz],
                                     [0, 0, 0, 1]])
        
        Projection_Matrix = np.dot(Intrinsic_Matrix, Extrinsic_Matrix)
        
        # get [X, Y, Z, 1]
        Z1 = 2.3
        for i in range(len(points)):
            world_point = [(points[i][0]-256)*Z1/256, (points[i][1]-256)*Z1/256, -Z1, 1]
            BEV_3D_points.append(world_point)
        
        #get front view [u, v, w]
        for i in range(len(BEV_3D_points)):
            tmp_P = np.dot(Projection_Matrix, np.array(BEV_3D_points[i]))
            front_3D_points.append(tmp_P.tolist())

        for i in range(len(front_3D_points)):
            front_3D_points[i][0] = front_3D_points[i][0] / front_3D_points[i][2]
            front_3D_points[i][1] = front_3D_points[i][1] / front_3D_points[i][2]
            front_3D_points[i][2] = front_3D_points[i][2] / front_3D_points[i][2]
        
        for i in range(len(front_3D_points)):
            u = front_3D_points[i][0]*256+256
            v = -1*front_3D_points[i][1]*256+256
            new_pixels.append([u, v])

        return new_pixels

    def show_image(self, new_pixels, img_name='task1_data/projection.png', color=(0, 0, 255), alpha=0.4):
        """
            Show the projection result and fill the selected area on perspective(front) view image.
        """
        new_image = cv2.fillPoly(
            self.image.copy(), np.int32([np.array(new_pixels)]), color)
        
        new_image = cv2.addWeighted(
            new_image, alpha, self.image, (1 - alpha), 0)

        cv2.imshow(
            f'Top to front view projection {img_name}', new_image)
        cv2.imwrite(img_name, new_image)
        print("save projection to front view")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return new_image


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        print(x, ' ', y)
        points.append([x, y])
        font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(img, str(x) + ',' + str(y), (x+5, y+5), font, 0.5, (0, 0, 255), 1)
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow('image', img)

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:
        print("right mouse")

        print(x, ' ', y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        # cv2.putText(img, str(b) + ',' + str(g) + ',' + str(r), (x, y), font, 1, (255, 255, 0), 2)
        cv2.imshow('image', img)


if __name__ == "__main__":

    pitch_ang = -90

    front_rgb = "task1_data/front_view_path.png"
    top_rgb = "task1_data/top_view_path.png"

    # click the pixels on window
    img = cv2.imread(top_rgb, 1)
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', click_event)
    # if any key then end mousecallback    
    cv2.waitKey(0)
    cv2.imwrite('task1_data/BEV_circle.png', img)
    print("save BEV with circle")
    cv2.destroyAllWindows()

    projection = Projection(front_rgb, points)
    new_pixels = projection.top_to_front(theta=pitch_ang)
    projection.show_image(new_pixels)
