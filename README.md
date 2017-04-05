# cwru_stereo_sync [![Build Status](https://travis-ci.org/cwru-robotics/cwru_stereo_sync.svg?branch=master)](https://travis-ci.com/cwru-robotics/cwru_davinci)

This package subscribes to an non-synchronized pair of camera topics. Artificially aligns the time stamps, then republishes the cameras.
The republished topics includes both and image and a camera info.

## Usage example

By default, the node subscribes to the unsynchronized camera pair:

"unsynced/left/image_raw"
"unsynced/right/image_raw"

The node then publishes to the synchronized camera pair:

"synced/left/image_raw"
"synced/right/image_raw"

In the launch directory, there is an example launch file for remapping the camera topics.


