# gimbalrotor

[English](README.md)

ロータをジンバルで傾けられるマルチロータです。機体は `robots/<airframe>/` にあります。このページでは `quad_with_dual_arm`(ジンバル付きロータ 4 つ、3 自由度のピッチアーム 2 本、前アームにフックモジュール)と、そのぶら下がり止まり(perching)動作を扱います。

## 起動

```bash
# 実機
roslaunch gimbalrotor bringup.launch airframe:=quad_with_dual_arm
# シミュレーション(Gazebo)
roslaunch gimbalrotor bringup.launch airframe:=quad_with_dual_arm real_machine:=false simulation:=true headless:=false
# シミュレーション(MuJoCo、止まり動作用のバーつき)
roslaunch gimbalrotor bringup.launch airframe:=quad_with_dual_arm real_machine:=false simulation:=true headless:=false \
  mujoco:=true mujoco_world:=quad_with_dual_arm_perching
# ジョイスティック(/gimbalrotor/joy)
roslaunch aerial_robot_base joy_stick.launch robot_name:=gimbalrotor
```

MuJoCo モデル(`mujoco/quad_with_dual_arm/`)は、`config/mujoco_model.yaml` から `catkin build` で生成されます。ワールド `worlds/quad_with_dual_arm_perching.xml` はこのモデルを読み込み、初期状態は `worlds/quad_with_dual_arm_perching.yaml` で与えます。

## 止まり動作: 計画 → 最適化 → 実行

| 手順 | コマンド | パラメータ | 出力 |
|---|---|---|---|
| 1. 計画 | `rosrun gimbalrotor plan_perching_motion.py` | `config/quad_with_dual_arm/perching_motion.yaml` | `config/quad_with_dual_arm/trajectories/perching_motion.json`(+ `.png`) |
| 2. 最適化 | `rosrun aerial_to_solver optimize_trajectory.py <1 の json>` | `aerial_to_solver/config/optimize_trajectory.yaml` | `perching_motion_optimized.json`(+ `.png`) |
| 3. 確認 | `roslaunch gimbalrotor perching_motion.launch trajectory_file:=<json>` | launch の引数 | RViz で再生 |
| 4. 実行 | `rosrun gimbalrotor perching_task.py __ns:=gimbalrotor [_takeoff:=true]` | `config/quad_with_dual_arm/follow_trajectory.yaml`、`perching_task.yaml` | ロボット(シミュレーションまたは実機) |

パラメータはすべて各 YAML にあり、個別に上書きできます。手順 1・2 は `--key value`、手順 4 は `_key:=value` で上書きします。手順 2 の詳細は `aerial_to_solver/README.ja.md` を見てください。

### スクリプト

`scripts/` は `planning/`（軌道計画）、`execution/`（軌道実行とタスク）、
`visualization/`（RViz 表示）、`simulation/`（MuJoCo モデルとワールド生成）に分けています。
`rosrun gimbalrotor <スクリプト名>.py` の呼び方は変わりません。

**`plan_perching_motion.py`** はバーまでの飛行を衝突判定付きで探索し、フック接触、ぶら下がり、離脱を IK で作ります。各フレームのロータ推力と接触力も静力学で計算し、JSON 軌道と、`--no-plot` を指定しなければ図を出力します。ROS master なしで実行できます。シーン、制約、時間配分、出力先は `perching_motion.yaml` にあり、コマンドラインで個別に上書きできます。衝突や静力学の失敗は集計して表示しますが、確認用の軌道出力は続けます。

**`trajectory_playback.py`** は計画または最適化した JSON 軌道から、ルートの TF、関節状態、ロータ推力、接触レンチ、接触状態で色分けした経路を RViz 向けに配信します。ロボットへの指令は送りません。`perching_motion.launch` から起動でき、再生速度、繰り返し、停止時間は `~speed`、`~loop`、`~pause` で設定します。

**`follow_trajectory.py`** は軌道ファイルの重心、機体姿勢、アーム関節の目標をロボットに送ります。必要ならモータ始動と離陸を行い、最初のフレームへ移動して静止したあと、`~end_frame` まで再生してその姿勢を保持します。接触の成否は判断せず、制御器の接触も切り替えません。ファイル、速度、送信周期、初期移動は `follow_trajectory.yaml` で設定します。ファイル中の推力とジンバル角は送信しません。

**`perching_task.py`** は同じ軌道追従に、フック接近前の収束待ち、低速接近、オペレータの OK/NG、接触の切り替え、その後の軌道再生を加えた SMACH の状態遷移です。NG のときは目標送信を止めて手動操縦に移り、接近開始点へ戻って再試行できます。判断と切り替えは `perching_task.yaml` で設定します。`after_hook: stop` ならフックを掛けた姿勢で終え、次のタスクへ引き継げます。

**`hanging_teleop.py`** は RViz の Interactive Marker または `~target` (`PoseStamped`) から後ろの手先目標を受けます。全身 IK で前アームのフックをバー上に保ちながら、ルート、機体ピッチ、両アームを動かし、関節・トルク・衝突の制約を考慮します。重心、姿勢、アーム関節の目標を `~rate` で送り、速度、制限、IK の重みは `hanging_teleop.yaml` で設定します。

**`contact_wrench_visualizer.py`** は全身制御器が公開する各接触力を購読し、接触設定から接触フレームの TF を配信し、RViz 用の `~contact` に力を再配信します。制御指令は出さず、`bringup.launch` から起動されます。

### 1. 計画(`perching_motion.yaml`)

コマンド用スクリプトは動作の各段階と軌道ファイル出力を定義します。パーチング固有のフレーム、IK タスク、接触方針は `src/gimbalrotor/perching_model.py`、タスク設定は `config/quad_with_dual_arm/perching_motion.yaml` に置きます。共通の IK、自由浮遊ベースの運動学、衝突シーン、軌道補間、静力学の基本処理は `aerial_motion_planning` パッケージに置き、ロータへの力配分と制約は `src/gimbalrotor/rotor_statics.py` に置きます。

動作の流れは、ホバリング → 飛行(RRTConnect)→ バーの手前で停止 → フックの開口部に沿って接近 → バーに荷重を移す → ぶら下がり(後アームは前に出した姿勢、`hang_rear_joints`)→ 荷重を戻す → 離脱、です。主なキーは次のとおりです。

- `bar_*`: シーン。`perching_motion.launch` と MuJoCo のワールドも同じ値にしてください。
- `start`、`hang_rear_joints`、`hook_tilt`、`approach`: タスク。
- `min_lift`、`max_effort_ratio`、`max_thrust_tilt`: 静力学(推力を上向きに保つ、トルクの余裕)。
- `reference_attitude`、`max_body_tilt`、`attitude_mask`、`attitude_weight`: IK と飛行での機体の姿勢。`attitude_mask` の各成分を 0 にすると、その軸を基準姿勢に固定します。
- `margin`、`bar_margin`: 飛行中の余裕(`bar_margin` はバーの上を越える前アームのもの)。
- `*_time`、`flight_*speed`: 時間配分(フレーム数 = 長さ × `rate`)。

### 4. 実行(`follow_trajectory.yaml`、`perching_task.yaml`)

#### 第1段階: フック手前までの変形飛行

```bash
rosrun gimbalrotor check_pre_hook_motion.py __ns:=gimbalrotor _takeoff:=true
```

既存のSMACHを `TAKEOFF → GO_TO_START → FLIGHT → PRE_HOOK_CHECK` まで実行します。最初の接触時刻から `approach_window` 秒前（現在の計画軌道では約11.4秒、228フレーム）の目標で停止し、重心位置・速度・アーム関節の収束を確認して終了します。フックへの接近や接触の有効化はしません。終了後もナビゲーションは最後の目標を保持するため、着陸は通常の操縦で行ってください。すでにホバリング中なら `_takeoff:=false` とします。既定では新しく計画した `perching_motion.json` を使い、`_trajectory_file:=...` で上書きできます。収束条件は `perching_task.yaml` にあります。

`perching_task.py` は SMACH の状態遷移です(`rosrun smach_viewer smach_viewer.py` で確認できます)。

```
TAKEOFF → GO_TO_START → FLIGHT → HOOK → JUDGE_HOOK ─OK→ AFTER_HOOK → succeeded
                                  ↑         │NG
                                RETURN ←OK─ TELEOP ─NG→ finished
```

- HOOK: バーの手前で、重心と関節が収束する(`converge_*`)まで待ちます。そのあと、接近を `approach_speed_scale` 倍速で流します。`converge_timeout` までに収束しなければ TELEOP に移ります。
- JUDGE_HOOK と TELEOP: オペレータの判断を待ちます。OK は R1(`ok_button`)、NG は R2(`ng_button`)です。トピックでも送れます。
  ```bash
  rostopic pub -1 /gimbalrotor/perching_task/ok std_msgs/Empty
  rostopic pub -1 /gimbalrotor/perching_task/ng std_msgs/Empty
  ```
- TELEOP: 目標を送らないので、スティックで操縦できます。OK を押すと、接近の開始点まで直線で戻ってやり直します。戻る前に、手動でフックをバーから外しておいてください。

`follow_trajectory.py` は指定フレームまでを再生するだけです。現行のサンプル軌道では最初の接触がフレーム 268 なので、接触前で止めるなら `_end_frame:=267` と指定できます。接近・収束待ち・オペレータの判断・接触の切り替えは `perching_task.py` にあり、その設定は `perching_task.yaml` (`_task_config:=...` で変更)に分けました。どちらも重心と姿勢を `uav/nav` に、アームの関節を `arm_joints_ctrl` に送ります。軌道ファイルの推力とジンバル角は使いません。

### 全身制御器(接触を考慮)

`bringup.launch whole_body:=true` で起動すると、gimbalrotor の制御器の配分を置き換えます。重心と姿勢の PID が求めるレンチを、全身の静力学の QP(摩擦円錐、関節トルクの上限、各ロータの最低揚力)で、ロータと有効な接触に配分します。接触が荷重を受けている間は、PID の積分を止めます。パラメータは `config/quad_with_dual_arm/GimbalrotorWholeBodyControl(_sim).yaml` の `controller/whole_body` にあり、接触の定義もここに書きます。接触は次のように切り替えます。

```bash
rosservice call /gimbalrotor/whole_body/contacts/hook_bar/activate true
```

有効にすると、`ramp_time` かけて荷重が移ります。`perching_task.py` はこれを自動で行います(`switch_contacts`)。フックはオペレータが成功を確認したとき、それ以外の接触は軌道ファイルで始まる・終わる時刻に切り替えます。

## ぶら下がり中の手先操作(全身 IK)

計画とぶら下がり操作の IK は、座標の範囲・マスク、任意の接触等式、任意の衝突確認を `aerial_motion_planning.ik.IKProblem` で共有します。計画では非線形最小二乗、操作では接触を厳密な等式にできる逐次 ProxQP を使います。点・方向・重心・トルクのタスクは各呼び出し側で指定します。

バーにぶら下がった状態で、`hanging_teleop.py` を使うと後ろのアームの手先を自由に動かせます。全身 IK(`src/gimbalrotor/hanging_ik.py`)はアームの面内で解きます。変数は、ルートの位置(機首方向と高さ)、機体のピッチ、アームの 6 関節です。フック点は等式制約でバー上に固定し、フックの角度と、フック直下の重心は重み付きのタスクとして保ちます。不等式制約は、関節の可動域、機体のピッチ(`max_pitch`)、後ろのアームの静的トルク(`max_effort_ratio`)、自己干渉です。届かない目標を与えると、手先は届く範囲で最も近い点で止まります。そのときもフックはバーから外れません。結果は毎周期、重心・姿勢・アーム関節の目標として送ります。

#### テレオペの実行手順(bringup 起動済み、ホバリング中)

1. バーまで飛んでフックを掛け、そこで止めます。
   ```bash
   rosrun gimbalrotor perching_task.py __ns:=gimbalrotor _after_hook:=stop
   ```
   計画した飛行を再生し、アプローチの手前で収束してから、ゆっくりバーに近づきます。「Hooked on the bar?」と聞かれたら R1(OK。`rostopic pub -1 /gimbalrotor/perching_task/ok std_msgs/Empty` でも可)を押します。フック接触が有効になり、フックを掛けた姿勢を保持したままノードが終了します。R2(NG)を押すとテレオペに移り、やり直せます。
2. 手先を動かします。
   ```bash
   rosrun gimbalrotor hanging_teleop.py __ns:=gimbalrotor
   ```
   ジョイスティック(L1 を押しながら。下記)、RViz のマーカー(InteractiveMarkers 表示を `/gimbalrotor/hanging_teleop/target_marker/update` に設定し、矢印をドラッグすると位置、リングを回すと向き)、またはトピックで目標を与えます。トピックの場合、手先の向きは orientation の x 軸で、位置も向きもアームの面に射影します。
   ```bash
   rostopic pub -1 /gimbalrotor/hanging_teleop/target geometry_msgs/PoseStamped \
     '{header: {frame_id: world}, pose: {position: {x: 0.6, y: 0.0, z: 0.7}, orientation: {y: 1.0, w: 0.0}}}'
   ```
3. バーから離れます。L1 と R1 を同時に 1 秒押し続けるか(`joy_unhook_button`、`joy_unhook_hold`)、次のトピックを送ります。
   ```bash
   rostopic pub -1 /gimbalrotor/hanging_teleop/unhook std_msgs/Empty
   ```
   IK で手先を開始時の位置(フックを掛けた姿勢)に戻します。次にフック接触を無効にし、`ramp_time` と `unhook_wait` の間その姿勢を保って、荷重を回転翼に戻します。最後に、姿勢を保ったままロボット全体をフックの開口に沿って `unhook_distance` だけ `unhook_speed` で動かします。計画のアプローチの逆向き(上へ、少し後ろへ)で、フックがバーから外れます。ノードはそこで終了し、ナビゲーションは最後の目標を保持します。着陸はジョイスティックで行います。手先が 30 秒以内に開始位置へ戻れない場合は、離脱せずに停止します(接触は有効のまま)。

IK はバーや環境との干渉を見ていません(見ているのは自己干渉だけです)。手先を前・上に出すと、IK は重心をフック直下に保つために機体をバーの方へ持ち上げることがあります。持ち上げ量は最大 0.3 m ほどで、プロペラがバーの数 cm 手前まで近づきます。手先が機体より上にあるときは、プロペラとバーの距離に注意してください。

ジョイスティックでは、L1 を押しながらスティックを動かします。左スティック上下で手先目標を機首方向に、右スティック上下で高さ方向に動かし、右スティック左右で手先の向きを回します。スティックをニュートラルに戻すか L1 を離すと、その時点の手先位置を目標にします。L1 を押している間、gimbalrotor のナビゲーションはスティックの飛行指令を 0 にする(`GimbalrotorNavigator::joyStickControl`)ので、重心は IK の指令だけで動きます。ボタンと軸は `hanging_teleop.yaml` の `joy_*`(PS4 の生のインデックス)で設定します。

フック点・アームの面・初期姿勢は、ノードを起動したときのロボットの状態から決めます。起動時には全身制御器のフック接触を有効にします(`activate_hook`)。パラメータ(速度、制限、IK の重み)は `config/quad_with_dual_arm/hanging_teleop.yaml` にあります。`~markers` には、フック点、IK の手先、IK の重心を表示します。

### 評価実験: バーの有無による手先精度の比較

両条件とも、計画上の同じ姿勢・同じ位置から始めます。開始点は軌道ファイルの最初のフック接触フレーム(`start_frame: hook`)で、どちらの条件でも `perching_task.py` でそこまで移動します。`hand_accuracy_test.py` は、そのフレームからハンギング IK で後腕の手先を世界座標の同じ目標へ順に動かします。IK は実測ではなく計画から始めるので、フック点は計画上のバー位置(世界座標で固定)になり、指令姿勢は両条件で同じです。違うのは、バーの有無と全身制御器のフック接触だけです。

#### 評価実験の実行手順(bringup 起動済み、ホバリング中)

バーあり(ファイル名は `perching`):

1. テレオペと同じようにフックを掛けて止めます(「Hooked on the bar?」で R1)。
   ```bash
   rosrun gimbalrotor perching_task.py __ns:=gimbalrotor _after_hook:=stop
   ```
2. 目標を実行します。
   ```bash
   rosrun gimbalrotor hand_accuracy_test.py __ns:=gimbalrotor _contact:=true
   ```
   スクリプトはフック接触を有効にし、ロボットが収束するまで開始フレームを保持します(`converge_*`)。その後、目標ごとに手先を動かし(MOVE、`settle_time` の SETTLE、`measure_time` の MEASURE)、最後に手先を開始位置へ戻します。`finish: unhook`(既定)の場合は、続けて開始フレームに収束させ、接触を無効にして荷重が回転翼に戻るのを待ち、アプローチを逆再生してフックをバーから抜きます。全体で約 1.5 分です。着陸はジョイスティックで行います。

バーなし(ファイル名は `hover`): バーを外し、接触を切り替えずに同じ動作をします。

```bash
rosrun gimbalrotor perching_task.py __ns:=gimbalrotor _after_hook:=stop _switch_contacts:=false   # フック時フレームで R1
rosrun gimbalrotor hand_accuracy_test.py __ns:=gimbalrotor _contact:=false
```

2 つの結果を比較します(ログは `~/.ros/hand_accuracy/` にあります)。

```bash
rosrun gimbalrotor compare_hand_accuracy.py ~/.ros/hand_accuracy/<hover>.csv ~/.ros/hand_accuracy/<perching>.csv
```

目標は手先の世界座標 `[x, z]` で、手先の向きのオフセットも任意で指定できます(`[x, z, dangle]`)。現在の計画では、フック時のルートは約 (0.00, 0.70)、手先は (0.64, 0.66) で水平に前を向き(計画の `hang_rear_joints` [0, 1.2, -1.2])、フック点は (0.00, 1.10) です。既定の 5 点は機体の前方で、開始時の手先の少し下から上までにあり、手先は水平のままです。この 5 点は、IK をオフラインで順番どおりに解いて確認済みで、すべて届き、機体は持ち上がらず、ロボットとバーの距離は 0.28 m 以上あります。IK はバーとの干渉を見ないので、目標を追加するときは同じように事前に確認してください(`hand_accuracy_test.yaml` のコメントを参照)。

ログには、実測の手先(推定したベースリンク姿勢と関節角からの順運動学)、IK の手先、各ロータの推力指令、フック接触力、全身制御 QP の関節トルクを記録します。設定は `config/quad_with_dual_arm/hand_accuracy_test.yaml`、IK のパラメータは `hanging_teleop.yaml` のものを使います。比較スクリプトは、目標ごとに手先誤差(全体・IK 分・制御分)、推力、接触力、トルク飽和を出力します。IK が制約するのは後腕の自重トルクだけです。バーの荷重を受ける前腕のトルクは QP だけが制約し、上限に達すると接触からロータへ荷重を移すか、QP が解けずに通常の配分に戻ります。推力を比較する前に `tau_sat` と `qp_fail` を確認してください。
