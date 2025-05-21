import numpy as np
import matplotlib.pyplot as plt
import sys

def read_csv(file_path: str) -> tuple:
    """
    Reads a CSV file and returns the data as a tuple of lists.

    Parameters:
    file_path (str): The path to the CSV file.

    Returns:
    tuple: A tuple containing two lists: the first list contains the first column of data,
           and the second list contains the second column of data.
    """
    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
    return data[:, 0].tolist(), data[:, 1].tolist(), data[:, 2].tolist(), data[:, 3].tolist()

def calculate_normal_distribution(x: float, mean: float, std_dev: float) -> float:
    """
    Calculates the normal distribution for a given x, mean, and standard deviation.

    Parameters:
    x (float): The value to calculate the normal distribution for.
    mean (float): The mean of the normal distribution.
    std_dev (float): The standard deviation of the normal distribution.

    Returns:
    float: The calculated normal distribution value.
    """
    return (1 / (std_dev * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mean) / std_dev) ** 2)

def plot_normal_distributions(means, std_devs, corrected_angles) -> None:
    """
    Plots normal distribution curves given the means and standard deviations.

    Parameters:
    means typing.List(float): The means of the normal distributions.
    std_devs typing.List(float): The standard deviations of the normal distributions.
    corrected_angles typing.List(float): The corrected angles for the normal distributions.

    Returns:
    None
    """
    # Generate x values
    x = np.linspace(min(means) - 4*max(std_devs), max(means) + 4*max(std_devs), 1000)
    
    plt.figure(figsize=(10, 6))
    plt.title('Normal Distribution Curves')
    plt.xlabel('Rotation angle (radiants)')
    plt.ylabel('Probability Density')
    plt.grid()

    for i, (mean, std_dev, corrected_angle) in enumerate(zip(means, std_devs, corrected_angles)):
        # Calculate the normal distribution
        y = calculate_normal_distribution(x, mean, std_dev)
        corrected_angle_probability = calculate_normal_distribution(corrected_angle, mean, std_dev)
        # Plot the normal distribution
        plt.plot(x, y, label=f"pc{i}; mean: {mean:.3f}, std dev: {std_dev:.3f}")
        plt.plot(corrected_angle, corrected_angle_probability, 'ro', label=f"pc{i}; corrected angle: {corrected_angle:.3f}")
    
    plt.legend()
    cwd = sys.path[0]
    plt.savefig(cwd + '/normal_distributions.png')

if __name__ == "__main__":
    ids, means, std_devs, corrected_angles = read_csv('/home/Referee/src/build/rotationAnglesAndStandardDeviations.txt')
    plot_normal_distributions(means, std_devs, corrected_angles)