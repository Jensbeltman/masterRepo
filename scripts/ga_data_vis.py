import json
import os

import matplotlib.pyplot as plt


def load_ga_result(path):
    with open(path, 'r') as file:
        return json.load(file)


def load_ga_results(directory):
    results = []
    for filename in os.listdir(directory):
        if filename.endswith(".json") or filename.endswith(".JSON"):
            print(os.path.join(directory, filename))
            with open(directory + '/' + filename, 'r') as file:
                results.append(json.load(file))

    return results


def plot_ga_result(ga_result):
    plt.figure()
    plt.plot(ga_result['best_chromosome_cost_history'])
    plt.title("GA cost history")
    plt.show()


def plot_ga_results(ga_results):
    plt.figure()
    for res in ga_results:
        plt.plot(res['best_chromosome_cost_history'])
    plt.title("GA cost history")
    plt.legend([str(i) for i in range(len(ga_results))])
    plt.show()


if __name__ == "__main__":
    ga_result = load_ga_results("../data/mutation_rate")

    final_costs = [res["best_chromosome_cost"] for res in ga_result]
    final_costs = [final_costs[i] for i in range(len(final_costs)) if (i % 5 == 0)]
    ga_result = [ga_result[i] for i in range(len(ga_result)) if (i % 5 == 0)]
    plt.plot(final_costs)

    plot_ga_results(ga_result)
