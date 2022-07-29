# 概要

Amberについてmoveitを使った例を示すためのROSパッケージです。

# Nodes

## motion_example_node

このサンプルでmove_groupインタフェースを利用した以下の内容を確認することが出来ます。

- goメソッドにより関節角度指定で動作させる(双腕）
- goメソッドによりPose指定で動作させる(双腕）
- compute_cartesian_pathメソッドにより予め軌道を計画し、executeメソッドにより実行させる（双腕）

## show_eefpose_node

このサンプルでは10Hzで左右の腕の手先位置をrospy.loginfo()で出力します。

## gripper_example_node

このサンプルでは、GripperCommandActionを利用して両手先の指先関節の開閉を行います。

## concurrent_motion_example_node

このサンプルでは、目標位置についてはmoveitのcompute_ikサービスで計算し、動作自体は
amber_ros_driverで提供されているset_joint_trajectoryサービスを利用することで、左右の
腕を同時に動かす例を提供します（現時点では、moveitでは同じロボットモデル内に定義された
複数のグループの軌道生成／実行に関して同時実行はサポートされていないため）。

# 動作確認方法

サンプルはそれぞれ以下のように実行できます。
前提としてamber_moveit_configのdemo_hw.launchが起動している必要があります。

``` bash
$ rosrun amber_moveit_examples motion_example_node
```

``` bash
$ rosrun amber_moveit_examples show_eefpose_node
```

``` bash
$ rosrun amber_moveit_examples gripper_example_node
```

``` bash
$ rosrun amber_moveit_examples concurrent_motion_example_node
```
