from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd



def main(run_id: int):
    run_folder = Path(__file__).parent.parent / 'outputs/'
    input_file = f'{str(run_folder)}/run{run_id}-accel-input.csv'
    output_file = f'{str(run_folder)}/run{run_id}-accel-output.csv'
    input_data = pd.read_csv(input_file, header=None)
    output_data = pd.read_csv(output_file, header=None)

    references = input_data.to_numpy().reshape((-1, 7))
    trackers = output_data.to_numpy().reshape((-1, 7))
    for i in range(7):
        plot_tracking(joint_id=i, reference=references, tracking=trackers)



def plot_tracking(joint_id: int, reference: np.ndarray, tracking: np.ndarray):
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(range(reference.shape[0]), reference[:, joint_id], label=rf'$\ddot q_{joint_id}^\ast$')
    ax.plot(range(tracking.shape[0]), tracking[:, joint_id], label=rf'$\ddot q_{joint_id}$')
    ax.set_ylabel(r'acceleration $(rad.s^{-2})$')
    ax.set_xlabel('timestep')
    ax.set_ylim([-2, 2])
    ax.set_xlim([0, reference.shape[0]])
    plt.legend()
    fig.savefig(f'Figures/acceleration-{joint_id}-tracking.png')
    # plt.show()




if __name__ == '__main__':
    run_id = 1
    main(run_id)