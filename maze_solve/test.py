import cv2
import os

# Set OpenCV to use headless mode
os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF'] = '0'
os.environ['OPENCV_VIDEOIO_PRIORITY_DSHOW'] = '0'

from .bot_map import bot_mapper
from .bot_path import DFS

bot_mapper_ = bot_mapper()
DFS_ = DFS()

def main():
    # Using forward slashes instead of backslashes to avoid escape sequence issues
    tiny = cv2.imread("/workspace/tiny.png", cv2.IMREAD_GRAYSCALE)

    # Displaying Tiny Maze
    cv2.namedWindow("tiny_maze",cv2.WINDOW_FREERATIO)
    cv2.imshow("tiny_maze",tiny)    
    
    # [Mapping] Applying the One Pass Algorithm to Convert Maze Image to Graph
    bot_mapper_.one_pass(tiny)
    print("** Graph Extracted **\n")
    bot_mapper_.Graph.displaygraph()
    print("\n** =============== **\n")
    cv2.waitKey(0)

    # [PathPlanning] Finding all paths to goal (End) using [DFS]
    start = bot_mapper_.Graph.start
    end = (3,4)
    
    paths = DFS_.get_paths(bot_mapper_.Graph.graph, start, end)
    
    # Displaying found paths
    print("Paths from {} to end {} is : \n {}".format(start,end,paths))
    


if __name__ == '__main__':
    main()