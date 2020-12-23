\page install Installation

Next Tutorial: \ref cppgetstarted

# Overview

This tutorial describes how to install Ignition Math on Linux, Mac OS X and
Windows via either a binary distribution or from source.

[Install](#install)

* [Binary Install](#binary-install)

* [Source Install](#source-install)

    * [Prerequisites](#prerequisites)

    * [Building from Source](#building-from-source)

# Install

We recommend following the [Binary Install](#binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Install

### Ubuntu Linux

Setup your computer to accept software from
*packages.osrfoundation.org*:

```{.sh}
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```{.sh}
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install Ignition Math:

```
sudo apt install libignition-math<#>-dev
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

#### Ubuntu Linux

The optional Eigen component of Ignition Math requires:

  * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Refer to the [Eigen Documentation](http://eigen.tuxfamily.org/index.php?title=Main_Page#Documentation) for installation instructions. On Ubuntu systems, `apt-get` can be used to install Eigen:

    ```
    sudo apt-get install libeigen3-dev
    ```

The optional Ruby tests of Ignition Math require:

 * [Ruby](https://www.ruby-lang.org/). Refer to the [Ruby Documentation](https://www.ruby-lang.org/downloads/) for installation instructions. On Ubuntu systems `apt-get` can be used to install Ubuntu Package `ruby-dev`:

    ```
    sudo apt-get install ruby-dev
    ```

  * [Swig](http://www.swig.org/). Refer to the [Swig Documentation](http://www.swig.org/download.html) for installation instructions. On Ubuntu systems `apt-get` can be used to install Swig:

    ```
    sudo apt-get install swig
    ```

#### Windows 10

First, follow the [ign-cmake](https://github.com/ignitionrobotics/ign-cmake) tutorial for building from source on Windows.

The optional Eigen component of Ignition Math requires:

  * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Refer to the [Eigen Documentation](http://eigen.tuxfamily.org/index.php?title=Main_Page#Documentation) for installation instructions. On Windows, we will use `conda` to instsall Eigen:

    conda install -c conda-forge eigen

### Building from source

#### Ubuntu

1. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-math -b ign-math<#>
    ```
    Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
    which version you need.

2. Install dependencies

    ```
    export SYSTEM_VERSION=bionic
    sudo apt -y install \
      $(sort -u $(find . -iname 'packages-'$SYSTEM_VERSION'.apt' -o -iname 'packages.apt') | tr '\n' ' ')
    ```

3. Configure and build

    ```
    cd ign-math; mkdir build; cd build; cmake ..; make
    ```

4. Optionally, install Ignition Math

    ```
    sudo make install
    ```

#### Windows

This assumes you have built [ign-cmake](https://github.com/ignitionrobotics/ign-cmake) from source using Conda, for which a Conda environment and a colcon workspace have been created.

1. Navigate to your ``condabin`` if necessary (find it in Anaconda Prompt, ``where conda``)), then activate the Conda environment:

    ```
    conda activate ign-ws
    ```

1. Navigate to the [colcon](https://colcon.readthedocs.io/en/released/) workspace where `ign-cmake` was built, and then clone the repository.
   We will be using a [colcon] workspace structure.

    ```
    cd ign_ws/src
    # This checks out the `main` branch. You can append `-b ign-cmake#` (replace # with a number) to checkout a specific version
    git clone https://github.com/ignitionrobotics/ign-math.git
    ```

1. Compile

    ```
    # Replace <#> with the numeric version you cloned
    colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install --packages-up-to ignition-math6
    ```
