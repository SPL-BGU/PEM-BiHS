css
<h1 align="center">On Parallel External-Memory Bidirectional Search</h1>
<p align="center">
<a href="https://github.com/SPL-BGU/BiHS-Consistent-F2E/blob/main/LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>

You can find the paper in [https://doi.org/10.3233/FAIA392, page 22](https://doi.org/10.3233/FAIA392).

The code for his repo is based on Prof. Nathan Sturtevant's HOG2 implementation which can be
found [here](https://github.com/nathansttt/hog2). <br/>
The code has been mainly run on the Ubuntu operating system using WSL and natively. <br/>

## Compiling the Code
The code was compiled using GCC and g++ versions 9.4.0. <br/>
You first need to download the dependencies, so run the following command for the headless version of the code.

```sh
# apt install build-essential
```

If you want the non-headless version of HOG2 and the code, or the above does not work, try the following:

```sh
# apt install build-essential libglu1-mesa-dev freeglut3-dev libsfml-dev
```

Then use the compile script in scripts/.


## Running the Experiments
To run the experiments which appeared in the paper, you need to run the the experiments script. This though would take a
very long time, so you should look into how to fragment those runs based on the info given by running the embhs file which 
should appear in src/bin/release/ after compilation.

```sh
./scripts/experiments.sh
```

## Generating Figures

Once you have all the results, you need to turn them into CSVs using the following, and then recreate the figures:

```sh
./scripts/csvs.sh
./scripts/figures.sh
```

Note that csvs.sh will run on all domains, so it might take a minute or two, depending on how many individual log files there
are. The more files, the longer it takes (you can simply concat files to speed this up).

The figures are saved into results, while the tables themselves are printed, they were then manually constructed in the
LaTex file. There is also a script to generate the tables for the appendix.

## Known Issues
Generating the figures in Linux seems to have a problem due to not having Times New Roman font. They specifically were
produced in Windows.

The interface to recreate the results was written after the fact, meaning it was not as heavily tested as the code for the papers.
This means that you might encounter issues with it. If you encounter any issue, find a bug, or need help, feel free to open an issue or contact Lior (the maintainer).

## Citation and Code Attribution
The new code for the paper can be found in
[src/apps/embhs](https://github.com/SPL-BGU/PEM-BiHS/tree/main/src/apps/embhs).<br/>
As said before, all of these rely on HOG2 which can be found [here](https://github.com/nathansttt/hog2). 

If you find our work interesting or the repo useful, please consider citing this paper:
```
@inproceedings{siag2024parallel,
  author       = {Lior Siag and
                  Shahaf S. Shperberg and
                  Ariel Felner and
                  Nathan R. Sturtevant},
  title        = {On Parallel External-Memory Bidirectional Search},
  booktitle    = {{ECAI}},
  series       = {Frontiers in Artificial Intelligence and Applications},
  volume       = {392},
  pages        = {4190--4197},
  publisher    = {{IOS} Press},
  year         = {2024}
}
```
