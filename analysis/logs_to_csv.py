import argparse
from pathlib import Path
from typing import Union
import pandas as pd


def parse_arguments():
    """Set up command-line argument parsing."""
    parser = argparse.ArgumentParser(description="Process log files and generate a CSV summary.")

    parser.add_argument(
        '-l', '--logs', required=True, help="Path to the directory containing log files.")

    parser.add_argument('-o', '--output', required=True, help="Path to the output CSV file.")

    return parser.parse_args()


def line_to_dict(line: str) -> dict[str, str]:
    return {k: v for part in line[4:].strip().split('; ') for k, v in [part.split(': ', 1)]}


def parse_file(file_path: Union[Path, str]) -> list[dict]:
    runlogs: list[dict] = []
    domain_dict = {}
    instance_dict = {}
    alg_dict = {}
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('[D]'):
                domain_dict = line_to_dict(line)
            elif line.startswith('[I]'):
                instance_dict = line_to_dict(line)
            elif line.startswith('[A]'):
                alg_dict = line_to_dict(line)
            elif line.startswith('[R]'):
                result_dict = line_to_dict(line)
                runlogs.append({**domain_dict, **instance_dict, **alg_dict, **result_dict})
    return runlogs


def parse_dir(dir_path: Union[str, Path]):
    dir_path: Path = Path(dir_path)
    runlogs: list[dict] = []
    for file_path in dir_path.rglob('*'):
        if file_path.is_file() and file_path.suffix in ['.log', '.txt', '.out']:
            runlogs.extend(parse_file(file_path))
    return runlogs


def main():
    args = parse_arguments()
    df = pd.DataFrame(parse_dir(args.logs))
    df.to_csv(args.output, index=False)


if __name__ == '__main__':
    main()
