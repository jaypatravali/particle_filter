import sys
from particle_filter import Particle_Filter
import argparse
import cv2
parser = argparse.ArgumentParser(description='Change parameters')

parser.add_argument('--particles', default=1000, type=int, metavar='N',
                          help='number of particles')
parser.add_argument('--DA', default="NN", type=str, metavar='N',
                          help='Search Algorithm: Choose NN or JCBB')
parser.add_argument('--noise',  action='store_false',
                          help='Add Measurement and Motion noise?   ')
parser.add_argument('--mode', default="sim", type=str, metavar='N',
                          help='Use sim or car data')
parser.add_argument('--play', default="disk", type=str, metavar='N',
                          help='Data from disk or realtime')
parser.add_argument('--init', default="set", type=str, metavar='N',
                          help='Particle initialized at random or set position')
parser.add_argument('--cam', default="zed", type=str, metavar='N',
                          help='Choose zed or PG')
parser.add_argument('--dynamics', default="odometry", type=str, metavar='N',
                          help='Choose dynamics model odometry or velocity')

args = parser.parse_args()


def main():
    print("args.noise", args.noise)
    pf = Particle_Filter(args.init, args.mode, args.play, args.noise, args.cam, args.dynamics)

    if args.play=='realtime':
        pf.process_realtime(args.particles)
    else:     
      pf.process_disk(args.particles)


if __name__ == "__main__":
    try:  
        main()
    except KeyboardInterrupt:
        print("Qutting...")
        cv2.destroyAllWindows()
        sys.exit()



# added new landmark poles after 70 in landmarks.txt for new loop
# not sure of 57,56, 58 coz hidden in trees near the 
# extra poles on building from 59-62