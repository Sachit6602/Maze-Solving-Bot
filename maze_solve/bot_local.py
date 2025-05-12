import cv2
import numpy as np

from .utilities import ret_smallest_obj, ret_largest_obj

class bot_localizer():
    def __init__(self):
        self.is_bg_extracted = False

        self.bg_model = []
        
        self.maze_og = []

        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transform_arr = []

        self.orig_rot = 0
        self.rot_mat = 0

        self.loc_car = 0

    
    # method to find convex hull enclosing contours(visual rep of land steepness)
    @staticmethod
    def rois_boundinghull(rois_mask,cnts):
        maze_enclosure = np.zeros_like(rois_mask)
        if cnts:
            cnts_ = np.concatenate(cnts)
            cnts_ = np.array(cnts_)
            cv2.fillConvexPoly(maze_enclosure, cnts_, 255)
        cnts_largest = cv2.findContours(maze_enclosure, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]# OpenCV 4.2
        hull = cv2.convexHull(cnts_largest[0])
        cv2.drawContours(maze_enclosure, [hull], 0, 255)
        return hull
    
    # to store position and size of maze, and precomputes the rotation matrix
    def update_frameofreference_parameters(self,X,Y,W,H,rot_angle):
        self.orig_X = X; self.orig_Y = Y; self.orig_rows = H; self.orig_cols = W; self.orig_rot = rot_angle # 90 degree counterClockwise
        self.transform_arr = [X,Y,W,H]
        # Rotation Matrix
        self.rot_mat = np.array(
                                [
                                 [ np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))],
                                 [-np.sin(np.deg2rad(self.orig_rot)) , np.cos(np.deg2rad(self.orig_rot))]
                                ]
                               )
        self.rot_mat_rev = np.array(
                                [
                                 [ np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))],
                                 [-np.sin(np.deg2rad(-self.orig_rot)) , np.cos(np.deg2rad(-self.orig_rot))]
                                ]
                               )
    
    
    # to store position and size of maze, and precomputes the rotation matrix
    def extract_bg(self, frame):
        #canny edge detection
        #convert to greyscale to detect edges
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, None, 3)

        #find external countours
        cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

        #draw all contours on the mask
        roi_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)        
        for ids in enumerate(cnts):
            cv2.drawContours(roi_mask, cnts, ids[0], 255, -1)
        
        #extract bg(remove the bot)
        min_cntr_ids = ret_smallest_obj(cnts)
        #ensure bot is found
        roi_noCar_mask =  roi_mask.copy()
        if min_cntr_ids!= -1:
            #remove bot from mask
            cv2.drawContours(roi_noCar_mask, cnts, min_cntr_ids, 0, -1)

            #extract mask for the bot
            car_mask =  np.zeros_like(roi_mask)
            cv2.drawContours(car_mask, cnts, min_cntr_ids, 255, -1)
            cv2.drawContours(car_mask, cnts, min_cntr_ids, 255, 3)
            #remove bot from frame
            noCar_mask =cv2.bitwise_not(car_mask)
            frame_car_remvd = cv2.bitwise_and(frame, frame, mask = noCar_mask)

            #create a flat bg with same base color
            base_clr = frame_car_remvd[0][0]
            Ground_replica = np.ones_like(frame)*base_clr
            #generate bg model
            self.bg_model = cv2.bitwise_and(Ground_replica, Ground_replica,mask = car_mask)
            self.bg_model = cv2.bitwise_or(self.bg_model, frame_car_remvd)

        
        # extracting the maze      
        # finding dimensions of hull enclosing largest contour
        hull = self.rois_boundinghull(roi_mask,cnts)
        [X,Y,W,H] = cv2.boundingRect(hull)

        # cropping maze_mask from the image
        maze = roi_noCar_mask[Y:Y+H,X:X+W]
        maze_occupencygrid = cv2.bitwise_not(maze)

        self.maze_og = cv2.rotate(maze_occupencygrid, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # maintaining frame of reference 
        self.update_frameofreference_parameters(X,Y,W,H,90)

        cv2.imshow("1a. rois_mask",roi_mask)
        cv2.imshow("1b. frame_car_remvd",frame_car_remvd)
        cv2.imshow("1c. Ground_replica",Ground_replica)
        cv2.imshow("1d. bg_model",self.bg_model)
        cv2.imshow("2. maze_og",self.maze_og)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    @staticmethod
    def get_centroid(cnt):
        M =  cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy =  int(M['m01']/M['m00'])
        return (cy ,cx)
    
    #localizes the robot relative to the maze
    def get_car_loc(self, car_cnt, car_mask):
        # get  centroid of the bot
        bot_cntr = self.get_centroid(car_cnt)
        # converting from point --> array to apply transforms
        bot_cntr_arr =  np.array([bot_cntr[1],bot_cntr[0]])
        # shift origin from sat_view -> maze
        bot_cntr_translated = np.zeros_like(bot_cntr_arr)
        bot_cntr_translated[0] = bot_cntr_arr[0] - self.orig_X
        bot_cntr_translated[1] = bot_cntr_arr[1]-self.orig_Y
        # applying rotation tranformation to bot_centroid to get bot location relative to maze
        bot_on_maze = (self.rot_mat @ bot_cntr_translated.T).T
        # translating Origin If neccesary (to get complete Image)
        rot_cols = self.orig_rows
        rot_rows = self.orig_cols
        bot_on_maze[0] = bot_on_maze[0] + (rot_cols * (bot_on_maze[0]<0) )  
        bot_on_maze[1] = bot_on_maze[1] + (rot_rows * (bot_on_maze[1]<0) )
        # Update the placeholder for relative location of car
        self.loc_car = (int(bot_on_maze[0]),int(bot_on_maze[1]))   



        
    #localize bot in current frame
    def localize_bot(self, curr_frame, frame_disp):

        if not self.is_bg_extracted:
            self.extract_bg(curr_frame)
            self.is_bg_extracted = True

        #foreground extraction
        change = cv2.absdiff(curr_frame, self.bg_model)
        change_gray = cv2.cvtColor(change, cv2.COLOR_BGR2GRAY)
        change_mask = cv2.threshold(change_gray, 15, 255, cv2.THRESH_BINARY)[1]
        car_mask, car_cnt = ret_largest_obj(change_mask)

        self.get_car_loc(car_cnt, car_mask)

        #marks bot with circle
        center, radii = cv2.minEnclosingCircle(car_cnt)
        car_circular_mask = cv2.circle(car_mask.copy(), (int(center[0]), int(center[1])), int(radii+(radii*0.4)), 255, 3)
        car_circular_mask = cv2.bitwise_xor(car_circular_mask, car_mask)
        frame_disp[car_mask>0]  = frame_disp[car_mask>0] + (0,64,0)
        frame_disp[car_circular_mask>0]  = (0,0,255)


        cv2.imshow("foreground(car)", car_mask)
        cv2.imshow("car_localized", frame_disp)