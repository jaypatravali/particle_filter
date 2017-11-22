import sys
from particle_filter import Particle_Filter
import argparse

parser = argparse.ArgumentParser(description='Change parameters')

parser.add_argument('--particles', default=1000, type=int, metavar='N',
                          help='number of particles')
parser.add_argument('--DA', default="NN", type=str, metavar='N',
                          help='Search Algorithm: Choose NN or JCBB')
parser.add_argument('--noise', default="False", type=bool, metavar='N',
                          help='Add Measurement and Motion Noise?   ')
parser.add_argument('--mode', default="car", type=str, metavar='N',
                          help='Use sim or car data')
parser.add_argument('--play', default="disk", type=str, metavar='N',
                          help='Data from disk or realtime')
parser.add_argument('--init', default="set", type=str, metavar='N',
                          help='Particle initialized at random or set position')
args = parser.parse_args()


def main():
    pf = Particle_Filter(args.init, args.mode,  args.play)

    if args.play=='realtime':
        pf.process_realtime(args.particles)
        
    pf.process_disk(args.particles)


if __name__ == "__main__":
    try:  
        main()
    except KeyboardInterrupt:
        print("Qutting...")
        sys.exit()
