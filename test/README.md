# Tests

[[_TOC_]]

# Build
- Build the package first:
```sh
$ catkin_make
# Note: This will not build the tests yet.
```

> **IMPORTANT**:
>
> Before building the tests, make sure the Hardware (ADRD2121 + ADI IMU) is **not connected**
>
> When you build the test, it will automatically execute, expect this to fail.

- Build and Run the tests
  + STEP 1
    ```sh
    [TERMINAL 1]
    $ roscore
    ```

  + STEP 2
    ```sh
    [TERMINAL 2]
    ## OPTION 1: Build all tests in your workspace
    $ catkin_make run_tests
    ## OPTION 2: Build all tests in your specific package
    $ catkin_make run_tests_adrd2121_imu
    ```

    *NOTE*
    - While it is possible to specify which test to build and execute via:
    ```sh
    $ catkin_make run_tests #<TAB> <TAB>

    ## OPTION 3: Build specific tests in your a specific package
    $ catkin_make run_tests_adrd2121_imu_rostest_test_test_16470_template.test
    ```
    - It was found that running OPTION 3 does not build the tests at all, and will result to failure.

  + STEP 3
    - Stop roscore in \[TERMINAL 1\]


# General Assumption/s

- The Hardware (ADRD2121 + ADI IMU) should be connected to the Host PC during execution of test;

# Tests/s

There are three tests:

## test_imu_data_raw

### Background
- Tests if the `/imu/data_raw` is being published;
- Tests if the `/imu/data_raw` linear_acceleration and angular_velocity is near expected.
- The Hardware should be **static** and **flat** on an even surface.
  + *TODO*: Add reference images

## test_param

### Background
- Tests if the parameters were set correctly

## test_hz

### Background
- Uses the reusable test node: [hztest](http://wiki.ros.org/rostest/Nodes#hztest)
- Tests if the `/imu/data_raw` is being published at expected frequency

## Configurations
- Per IMU, there are 2 different ```yaml`` files (i.e. 2 different configurations) to be tested.
  1. config/adis<prod_id>_template.yaml
  2. test/config/adis<prod_id>_configA.yaml

## .test files
There is 1 `.test` fileper configuration that executes `test_imu_data_raw`, `test_param`, and `test_hz`:

**NOTE:**
In each of the `.test` file,
1. The config/<imu product>_template.yaml file is loaded
2. The `adrd2121_imu_node` will be started first, before the test.

## Execute tests
**NOTE:** As long as the tests were built correctly, the following commands can be used to run each test.

```sh
$ rostest --text adrd2121_imu test_<imu product>_template.test
# The '--text'  outputs the logs of the test in the screen;

# Example for ADIS16470
$ rostest --text adrd2121_imu test_16470_template.test
```

**Limitation:**
- Each test file could one be executed 1 at a time, else all tests will fail.
- This is because the `adrd2121_imu_node` will try to access `/dev/ttyACM0` for each test.
- *For improvement:* Set a fixed device name for each IMU to be able to run each `.test` file in parallel.
