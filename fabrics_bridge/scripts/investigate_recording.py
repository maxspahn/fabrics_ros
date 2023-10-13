import pickle
import sys
from record import Recording

if __name__ == "__main__":
    with open(sys.argv[1], 'rb') as f:
        recording: Recording = pickle.load(f)

    print(recording)
