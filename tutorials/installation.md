\page install Installation

Next Tutorial: \ref cppgetstarted

# Install

We recommend following the Binary Install instructions to get up and running as quickly and painlessly as possible.

The Source Install instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

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

First, follow the [ign-cmake](https://github.com/ignitionrobotics/ign-cmake) tutorial for installing Conda, Visual Studio, CMake, etc., prerequisites, and creating a Conda environment.

The optional Eigen component of Ignition Math requires:

  * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Refer to the [Eigen Documentation](http://eigen.tuxfamily.org/index.php?title=Main_Page#Documentation) for installation instructions. On Windows, we will use `conda` to install Eigen:

    ```
    conda install -c conda-forge eigen
    ```

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

1. Navigate to ``condabin`` if necessary to use the ``conda`` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of ``condabin`` in Anaconda Prompt, ``where conda``).
   Activate the Conda environment created in the prerequisites:

    ```
    conda activate ign-ws
    ```

1. Install dependencies

   View the list of dependencies, replacing `<#>` with the desired version:
    ```
    conda search libignition-math<#> --channel conda-forge --info
    ```
   See the [Conda release repository](https://github.com/conda-forge/libignition-math4-feedstock) for more information.

   Install dependencies, replacing `<#>` with the desired version:
    ```
    conda install libignition-cmake<#> --channel conda-forge
    ```

1. Navigate to where you would like to build the library, and clone the repository.

    ```
    # This checks out the `main` branch. You can append `-b ign-cmake#` (replace # with a number) to checkout a specific version
    git clone https://github.com/ignitionrobotics/ign-math.git
    ```

1. Configure and build

    ```
    cd ign-math
    mkdir build
    cd build
    cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
    cmake --build . --config Release
    ```

1. Optionally, install Ignition Math

    ```
    cmake --install . --config Release
    ```
