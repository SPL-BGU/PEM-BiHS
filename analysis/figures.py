from typing import Optional

import matplotlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter


def human_format(num):
    num = float('{:.3g}'.format(num))
    magnitude = 0
    while abs(num) >= 1000:
        magnitude += 1
        num /= 1000.0
    return '{}{}'.format('{:f}'.format(num).rstrip('0').rstrip('.'), ['', 'K', 'M', 'B', 'T'][magnitude])


def format_number(number):
    if number == int(number):
        return f'{number:.0f}'
    return f'{number:.2f}'


def billions_formatter(x, pos):
    return f'{x / 1e9:.0f}'


def trillions_formatter(x, pos):
    return f'{x / 1e12:.0f}'


def ifint_formatter(x, pos):
    return int(x) if int(x) == x else x


def generate_ablation_graph(df, figure_path):
    df = df.groupby(['alg', 'threads'], as_index=False).agg({'elapsed': 'mean'})
    df['elapsed'] = df['elapsed'] / 3600
    df = df.sort_values(by='threads')
    xvalues = sorted(df['threads'].unique().tolist())
    aida_yvalues = df[df['alg'] == 'AIDA*']['elapsed'].tolist()
    pembae_yvalues = df[df['alg'] == 'PEM-BAE*']['elapsed'].tolist()

    # Set general fonts
    fontsize = 25
    font = {'family': 'Times New Roman', 'size': fontsize}
    matplotlib.rc('font', **font)
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42

    # Create a figure of size
    plt.figure(figsize=(8, 6))

    # Generate plots
    colors = ['blue', 'orange', 'red', 'green', 'c', 'm']
    linestyles = ['-', '--', ':', '-.', '-']
    markers = ['o', 's', 'd', '^', 'v']

    plt.plot(xvalues, pembae_yvalues, label='PEM-BAE*', linewidth=4, color='blue', linestyle='-', marker='o',
             markersize=12)
    plt.plot(xvalues, aida_yvalues, label='AIDA*', linewidth=4, color='orange', linestyle='--', marker='s',
             markersize=12)

    # X-axis
    plt.xticks([1, 16, 32, 48, 64, 80, 96])
    plt.xlabel('Number of Threads', weight='bold', fontsize=fontsize)
    plt.xlim(0, 96)

    # Y-axis
    plt.yscale('log')
    y_range = np.logspace(np.log10(0.1), np.log10(15), num=10)
    ytick_labels = ['{:.1f}'.format(val, int(np.log10(val))) for val in y_range]
    ytick_labels[0] = '0'  # Update the first tick to be 0 (even though it is 0.1), just looks nicer
    plt.yticks(y_range, ytick_labels)
    plt.ylim(0, 15)
    plt.ylabel('Average Time (Hours)', weight='bold', fontsize=fontsize)

    for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
        label.set_fontweight('bold')
        label.set_fontsize(fontsize)

    # Legend
    legend = plt.legend()
    for text in legend.get_texts():
        text.set_fontsize(fontsize)
        text.set_fontweight('bold')

    plt.grid(True)
    plt.tight_layout()

    if figure_path:
        plt.savefig(figure_path, bbox_inches='tight')
    else:
        plt.show()


def generate_graph(df: pd.DataFrame, out_path: Optional[str], col: str, ylabel: str, algs: list[str],
                   styles: dict[str, tuple[str, str, str]], ylabel_formatter=None, ylimmax: float = -1,
                   figsize: tuple[float, float] = (8, 6), yfactor: float = 1):
    plt.tight_layout()
    # Filter out irrelevant algorithms
    df = df[df['alg'].isin(algs)]
    # If two instances have the same solution length, average over them
    df = df.groupby(['alg', 'solution'])[col].mean().reset_index()
    # Sort based on increasing length
    df.sort_values(by='solution', inplace=True, ignore_index=True)

    # Extract x-values, i.e., solution lengths. PEM-BAE* is chosen as it always appears
    x_values = df[df['alg'] == 'PEM-BAE*']['solution'].tolist()

    # For each algorithm, make a list of the y-values, i.e., the col, and also factor it by y (as time is in hours)
    y_values = []
    for alg in algs:
        y_values.append([_ / yfactor for _ in df[df['alg'] == alg][col].tolist()])

    # Generate layout
    plt.clf()
    plt.figure(figsize=figsize)
    plt.tight_layout(pad=0.01)

    # Set fonts
    font = {'family': 'Times New Roman', 'size': 20}

    matplotlib.rc('font', **font)

    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42

    for i, alg in enumerate(algs):
        plt.plot(x_values, y_values[i], color=styles[alg][1], linewidth=4.5, linestyle=styles[alg][2],
                 label=(styles[algs[i]][0] + ' (' + human_format(sum(y_values[i]) / len(y_values[i]))) + ')')

    plt.xlabel('Instances Ordered by Solution Length', weight='bold', fontsize=25)
    plt.ylabel(ylabel, weight='bold', fontsize=25)

    plt.xlim(min(x_values), max(x_values))
    plt.ylim(ymin=0)
    if ylimmax > 0:
        plt.ylim(ymax=ylimmax)

    # Set labels and ticks to bold and size
    plt.gca().set_yticklabels(plt.gca().get_yticks(), weight='bold')
    plt.gca().set_xticklabels(plt.gca().get_xticks(), weight='bold')
    plt.xticks(fontsize=27)
    plt.yticks(fontsize=27)

    # Handle tick formatting
    if ylabel_formatter is not None:
        plt.gca().yaxis.set_major_formatter(FuncFormatter(ylabel_formatter))

    plt.gca().xaxis.set_major_formatter(FuncFormatter(ifint_formatter))

    # Generate legend
    legend = plt.legend(loc='upper left', prop={'weight': 'bold'})
    for text in legend.get_texts():
        text.set_fontsize(30)
        text.set_fontweight('bold')

    if out_path:
        plt.savefig(out_path, bbox_inches='tight')
    else:
        plt.show()


def main():
    styles = {
        'PEMM': ('PEM-MM', '#EE3377', '-.'),
        'PEM-A*': ('PEM-rA*', '#EE7733', '--'),
        'PEM-RA*': ('PEM-A*', '#009988', ':'),
        'PEM-BAE*': ('PEM-BAE*', 'blue', '-'),
        'AIDA*': ('AIDA*', '#EE7733', '--'),
        'RAIDA*': ('rAIDA*', '#009988', ':')
    }
    stp5_df = pd.read_csv('results/csvs/stp5.csv')

    stp5_96_df = stp5_df[stp5_df['threads'] == 96]
    generate_graph(stp5_96_df, 'results/figures/stp5_expanded.pdf',
                   'expanded', 'Expansions (Trillions)',
                   algs=['RAIDA*', 'AIDA*', 'PEM-BAE*'], styles=styles,
                   ylabel_formatter=trillions_formatter,
                   ylimmax=7000000000000)

    generate_graph(stp5_96_df, 'results/figures/stp5_time.pdf',
                   'elapsed', 'Time (Hours)',
                   algs=['RAIDA*', 'AIDA*', 'PEM-BAE*'], styles=styles,
                   ylimmax=120, figsize=(8, 6.34),
                   ylabel_formatter=ifint_formatter,
                   yfactor=3600)

    toh_df = pd.read_csv('results/csvs/toh.csv')
    generate_graph(toh_df, 'results/figures/toh_expanded.pdf',
                   'expanded', 'Expansions (Billions)',
                   algs=['PEMM', 'PEM-RA*', 'PEM-A*', 'PEM-BAE*'], styles=styles,
                   ylabel_formatter=billions_formatter,
                   ylimmax=20000000000)

    generate_graph(toh_df, 'results/figures/toh_time.pdf',
                   'elapsed', 'Time (Hours)',
                   algs=['PEMM', 'PEM-RA*', 'PEM-A*', 'PEM-BAE*'], styles=styles,
                   ylabel_formatter=ifint_formatter,
                   yfactor=3600)

    condition = (stp5_df['alg'].str.startswith('PEM-BAE') | stp5_df['alg'].str.startswith('AIDA')) & (
            stp5_df['solution'] == 100)
    stp5_ablation_df = stp5_df[condition]
    generate_ablation_graph(stp5_ablation_df, 'results/figures/ablation.pdf')


if __name__ == '__main__':
    main()
