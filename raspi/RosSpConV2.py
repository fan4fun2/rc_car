#!/usr/bin/env python
# -*- coding: utf-8 -*-

#==============================================================================
# ラジコン・カーをROSから制御するプログラム                RosSpCon.py Ver.2.3
#                                                       fan4fun2rc 29 Apr. 2022
#------------------------------------------------------------------------------
#　V1:ラジコン・カーの速度制御をRotaryEncoder.pyを使用したスレッド処理で実現した
#　   GuiRemoCon.pyを作成し速度設定とロータリ・エンコーダからの速度をGUIで
#　   表示させたが、GUI部にROSのリモコンを使用するためROS環境に移行する。
#　V2:ＰＩＤ制御の係数をテストするために専用のＧＵＩを作成し動作中に変更できるよう
#　   RosBrdgGui.pyとトピックで通信するように改変する。
#
#【仕様】
#　・速度情報はRotaryEncoder.pyからロータリ・エンコーダの積算値を入力する
#　・目標速度の指定はROS1標準の/cmd_velを購読し設定する
#　・ＰＩＤ制御の係数は/param_infoを購読して変更できるようにする
#　・目標速度に対するＰＩＤ制御とＰＷＭ直接駆動が/drive_infoで変更できるようにする
#　・現在の速度情報は左右のタイヤの速度を/speed_infoとして配信する
#　・速度制御はサブスレッドで実行し100ms周期でPID制御を行う
#　・メインスレッドはROSの終了までrospy.spin()が実行される
#
#【関数概要】
#　１．左右の車輪を目的の速度に制御するスレッド処理関数：speedControl()
#　２．トピック/cmd_velを受信時のコールバック関数：twistCallback(cmd_vel)
#　３．トピック/drive_infoを受信時のコールバック関数：driveCallback(roi)
#　４．トピック/param_infoを受信時のコールバック関数：paramCallback(roi)
#　５．メイン処理関数：main()
#
#【リビジョン・ヒストリ】
#　・V1.0-2022/03/06:コーディング開始
#　・V2.0-2022/03/13:テスト用ＧＵＩのRosBrdgGui.py対応に修正
#　・V2.1-2022/03/14:目標速度制御を行うときのＰＷＭにオフセットを追加
#　・V2.2-2022/03/17:デバッグログにオールゼロが５回続いたらログを出さないように変更
#　　　　　　　　　　　スタート時にロータリ・エンコーダのカウンタをクリアしておく
#　・V2.3-2022/04/29:ノード名が分かりにくいので「radicon_car」から「rc_car_pi」に変更
#==============================================================================

#------------------------------------------------------------------------------
#　インポートするライブラリの宣言
#------------------------------------------------------------------------------
import rospy                            # ROSのPythonライブラリ
from geometry_msgs.msg import Twist     # /cmd_vel用メッセージ
from sensor_msgs.msg import RegionOfInterest as ROI     # /xxx_infoに代用
import RPi.GPIO as GPIO
import sys
import threading
import time
import RotaryEncoder as re

#------------------------------------------------------------------------------
#　ハードウェアに接続した入出力ポートをBCM番号で定義する
#　＜注：リモコンの入力ポートは連番がとする必要がある＞
#------------------------------------------------------------------------------
OT_L_F = 19     # 左モーターの正転駆動出力GPIOポート
OT_L_R = 13     # 左モーターの逆転駆動出力GPIOポート
OT_R_F = 18     # 右モーターの正転駆動出力GPIOポート
OT_R_R = 12     # 右モーターの逆転駆動出力GPIOポート

#------------------------------------------------------------------------------
#　モジュール内で使用する定数と変数の定義
#------------------------------------------------------------------------------
PWM_FREQ = 200  # PWMの周波数を200Hzに設定

Ltarget = 0.0   # 左車輪の目標速度[mm/sec]
Rtarget = 0.0   # 右車輪の目標速度[mm/sec]

MMP = 3.119776  # 1パルスで進む距離（計算式はRotaryEncoder.pyを参照）

flag = True     # スレッド処理の終了フラグ
plf = None      # 左前進用のPWMポートへのオブジェクトID
prf = None      # 右前進用のPWMポートへのオブジェクトID
plr = None      # 左後退用のPWMポートへのオブジェクトID
prr = None      # 右後退用のPWMポートへのオブジェクトID

#------------------------------------------------------------------------------
# ここからROS用の定数と変数
# ①AndroidのROSアプリTeleopは、
#   ・前進最大がlinear.x=1.0[m/s]
#   ・後退最大がlinear.x=-1.0[m/s]
#   ・左回転最大がangular.z=1.0[rad/s]
#   ・右回転最大がangular.z=-1.0[rad/s]
#   なので、speedControl()の目標速度[mm/sec]への変換と感度調整を行う
#   定数を設定する
# ②車輪の間隔（トレッド）が106mmなので、angular.z=2πで１秒間に
#   左のタイヤが106ｘπ＝333.0088mm後退方向に回転し、
#   右のタイヤが333.0088mm前進方向に回転することになる
#   よって、アプリの最大値では106ｘπ/2π=56[mm/sec]となる
L_SCALE = 0.5*1000      # 最大並進速度を50cm/secとする
A_SCALE = 0.5*1000      # 最大回転速度で50cm/secとする
Lpwm = 0                # 左駆動のＰＷＭ量[%]
Rpwm = 0                # 右駆動のＰＷＭ量[%]
Ltw  = 1                # 左駆動が目標速度制御[1]かＰＷＭ直接駆動[0]かのフラグ
Rtw  = 1                # 右駆動が目標速度制御[1]かＰＷＭ直接駆動[0]かのフラグ

Ll   = 15               # 左ＰＩＤ制御の目標速度に応じた駆動係数
Lp   = 10               # 左ＰＩＤ制御の偏差に対する比例係数
Li   = 10               # 左ＰＩＤ制御の偏差に対する積分係数
Ld   = 0                # 左ＰＩＤ制御の偏差に対する微分係数
Lo   = 10               # 左ＰＷＭ駆動に対するオフセット値

Rl   = 15               # 右ＰＩＤ制御の目標速度に応じた駆動係数
Rp   = 10               # 右ＰＩＤ制御の偏差に対する比例係数
Ri   = 10               # 右ＰＩＤ制御の偏差に対する積分係数
Rd   = 0                # 右ＰＩＤ制御の偏差に対する微分係数
Ro   = 10               # 右ＰＷＭ駆動に対するオフセット値

#==============================================================================
# １．左右の車輪を目的の速度に制御するスレッド処理関数：speedControl()
#------------------------------------------------------------------------------
#　・ローカル変数の定義と初期化を行う
#　・ロータリーエンコーダー用のGPIOの設定やモジュール変数の定義と初期化を行う
#　・100ms毎に目標速度と現速度を比較してPWMを制御するループ
#　　－目標速度との誤差に応じたPID制御を行うために、
#　　　誤差Pと誤差の積分Iと誤差の微分Dを求める
#　　－10回分で平均された秒速を計算する[mm/sec]
#　　－Raspiの負荷を減らすため速度情報に変化があった時のみ送信する
#　　－PID制御の実行
#　　－モーターに指示するPWM値がプラスなら正転でマイナスなら逆転
#==============================================================================
def speedControl():
    global Ltarget, Rtarget, Lpwm, Rpwm, Ltw, Rtw
    global Ll, Rl, Lp, Rp, Li, Ri, Ld, Rd
    #
    # ローカル変数の定義と初期化を行う
    #
    l = [0] * 10    # 左の10回分のパルス数をメモルリスト
    r = [0] * 10    # 右の10回分のパルス数をメモルリスト
    Lsum = 0        # 左の10回分のパルス数を足した値
    Rsum = 0        # 右の10回分のパルス数を足した値

    lI = [0.0] * 10 # 左の10回分の誤差をメモルリスト
    rI = [0.0] * 10 # 右の10回分の誤差をメモルリスト
    lIsum = 0.0     # 左の10回分の誤差を足した値
    rIsum = 0.0     # 右の10回分の誤差を足した値

    i = 0           # 10回カウンタ
    ld = 0.0        # 左車輪へのデューティーサイクル
    rd = 0.0        # 右車輪へのデューティーサイクル

    preLsp = 1.0    # 1回前の左速度を保持する、初期値はノンゼロで
    preRsp = 1.0    # 1回前の右速度を保持する
    #
    # ロータリーエンコーダー用のGPIOの設定やモジュール変数の定義と初期化を行う
    #
    re.init()
    stm = time.time()
    rospy.loginfo("制御スレッドを開始しました。")
    log5 = 0        # デバッグ用のログでオールゼロをカウント(V2.2)
    ccnt = re.get_pls_cnt()             # スタート時にカウンタをクリアしておく(V2.2)
    #
    # 100ms毎に目標速度と現速度を比較してPWMを制御するループ
    #　メインスレッドが終了したときに停止できるようにフラグでチェックする
    #
    while flag:
        time.sleep(0.1)
        ccnt = re.get_pls_cnt()         # 前回からのパルスカウント数を左右リストで取得する
        Lsum = Lsum - l[i] + ccnt[0]    # 10回前の値を削除して今回分を足す
        Rsum = Rsum - r[i] + ccnt[1]
        l[i] = ccnt[0]
        r[i] = ccnt[1]
        #
        # 目標速度との誤差に応じたPID制御を行うために、
        #　誤差Pと誤差の積分Iと誤差の微分Dを求める
        #
        lP = Ltarget - ccnt[0]*10*MMP   # 今回の誤差を求める
        rP = Rtarget - ccnt[1]*10*MMP
        lIsum = lIsum - lI[i] + lP      # 10回前の値を削除して今回分を足す
        rIsum = rIsum - rI[i] + rP      # 積分値は誤差の10回分
        lI[i] = lP
        rI[i] = rP
        lD = lP - lI[(i-1)%10]          # 微分値は前回との誤差の差
        rD = rP - rI[(i-1)%10]
        i += 1
        i %= 10
        #
        # 10回分で平均された秒速を計算する[mm/sec]
        #
        lsp = Lsum * MMP                # MMPは1パルス当たりの走行距離(mm)
        rsp = Rsum * MMP
        #
        # Raspiの負荷を減らすため速度情報に変化があった時のみ送信する
        #　・sensor_msgs/RegionOfInterest.msgを使用して（独自型を作成せず）
        #　　‐bool do_rectify ：未使用、常にFalse
        #　　‐uint32 width    ：左車輪の速度[mm/s]の絶対値
        #　　‐uint32 height   ：右車輪の速度[mm/s]の絶対値
        #　　‐uint32 y_offset ：左車輪が前進方向なら０、後退方向なら１にエンコード
        #　　‐uint32 x_offset ：右車輪が前進方向なら０、後退方向なら１にエンコード
        #
        if lsp != preLsp or rsp != preRsp:
            preLsp = lsp
            preRsp = rsp
            spd = ROI()
            spd.width    = lsp if lsp >= 0 else -lsp
            spd.y_offset = 0   if lsp >= 0 else 1
            spd.height   = rsp if rsp >= 0 else -rsp
            spd.x_offset = 0   if rsp >= 0 else 1
            speedControl.pub.publish(spd)
        #
        # ここから、左が目標速度制御[1]かＰＷＭ直接駆動[0]かの処理を追加(V2.0)
        #
        if Ltw == 1:    # 左駆動が目標速度制御
            #
            # PID制御の実行
            #   駆動量＝目標速度Ｘ駆動係数＋偏差Ｘ比例係数
            #           ＋偏差の積分値Ｘ積分係数＋偏差の微分値Ｘ微分係数
            #
            ld = int(Ltarget*Ll/100 + lP*Lp/100 + lIsum*Li/1000 + lD*Ld/100)      
            #
            # 駆動力に10%のオフセットを付ける
            #
            if   ld > 0:    ld += Lo
            elif ld < 0:    ld -= Lo
            if   ld > 100:  ld =  100
            elif ld < -100: ld = -100
        else:
            ld = Lpwm
        #
        # ここから、右が目標速度制御[1]かＰＷＭ直接駆動[0]かの処理を追加(V2.0)
        #
        if Rtw == 1:    # 右駆動が目標速度制御
            #
            # PID制御の実行
            #   駆動量＝目標速度Ｘ駆動係数＋偏差Ｘ比例係数
            #           ＋偏差の積分値Ｘ積分係数＋偏差の微分値Ｘ微分係数
            #
            rd = int(Rtarget*Rl/100 + rP*Rp/100 + rIsum*Ri/1000 + rD*Rd/100)
            #
            # 駆動力に10%のオフセットを付ける
            #
            if   rd > 0:    rd += Ro
            elif rd < 0:    rd -= Ro
            if   rd > 100:  rd =  100
            elif rd < -100: rd = -100
        else:
            rd = Rpwm
        #
        # モーターに指示するPWM値がプラスなら正転でマイナスなら逆転
        #
        if ld == 0:
            plf.ChangeDutyCycle(0)
            plr.ChangeDutyCycle(0)
        elif ld > 0:
            plf.ChangeDutyCycle(ld)
            plr.ChangeDutyCycle(0)
        else:
            plf.ChangeDutyCycle(0)
            plr.ChangeDutyCycle(0-ld)

        if rd == 0:
            prf.ChangeDutyCycle(0)
            prf.ChangeDutyCycle(0)
        elif rd > 0:
            prf.ChangeDutyCycle(rd)
            prr.ChangeDutyCycle(0)
        else:
            prf.ChangeDutyCycle(0)
            prr.ChangeDutyCycle(0-rd)
        #
        # デバッグ用のログでオールゼロが５回続いたらログを出さないように修正(V2.2)
        #
        if Ltarget != 0 or ccnt[0] != 0 or ld != 0 or Rtarget != 0 or ccnt[1] != 0 or rd != 0:
            print("%6.2f, %8.2f,%8.2f,%4d, %8.2f,%8.2f,%4d"
                % (time.time() - stm, Ltarget, ccnt[0]*10*MMP, ld,
                                        Rtarget, ccnt[1]*10*MMP, rd))
            log5 = 0
        elif log5 < 5:
            print("%6.2f, %8.2f,%8.2f,%4d, %8.2f,%8.2f,%4d"
                % (time.time() - stm, Ltarget, ccnt[0]*10*MMP, ld,
                                        Rtarget, ccnt[1]*10*MMP, rd))
            log5 += 1
    #
    rospy.loginfo("制御スレッドを終了しました。")


#==============================================================================
# ２．トピック/cmd_velを受信時のコールバック関数：twistCallback(cmd_vel)
#   ＜引数 cmd_vel：並進速度(m/s)と回転速度(rad/s)を指定する＞
#------------------------------------------------------------------------------
#　・引数から左右の車輪速度を合成しモジュール変数に設定する
#==============================================================================
def twistCallback(cmd_vel):
    global Ltarget, Rtarget
    #
    # 並進速度は左右の車輪は同じ値だが、回転速度は左が正の値なので
    #　左車輪は後退方向(-)で右車輪は前進方向(+)となる
    #
    lx, az = cmd_vel.linear.x * L_SCALE, cmd_vel.angular.z * A_SCALE
    Ltarget = lx - az
    Rtarget = lx + az
    #
    # 目標速度制御[1]かＰＷＭ直接駆動[0]かのフラグを目標速度制御に設定
    #
    Ltw = 1
    Rtw = 1


#==============================================================================
# ３．トピック/drive_infoを受信時のコールバック関数：driveCallback(roi)
#   ＜引数 roi：左/右の車輪の駆動情報を下記のようにエンコードしている＞
#------------------------------------------------------------------------------
#　・sensor_msgs/RegionOfInterest.msgを使用して（独自型を作成せず）
#　　‐bool do_rectify ：左ならTrue、右ならFalseにエンコード
#　　‐uint32 width    ：目標速度なら１、ＰＷＭなら０にエンコード
#　　‐uint32 height   ：目標速度またはＰＷＭの整数をエンコード
#　　‐uint32 y_offset ：前進方向なら０、後退方向なら１にエンコード
#==============================================================================
def driveCallback(roi):
    global Ltarget, Rtarget, Lpwm, Rpwm, Ltw, Rtw
    #
    # 目標速度による速度制御とＰＷＭによる直接駆動をグローバル変数で指定する
    #
    if roi.do_rectify == True:      # 左
        if roi.width == 1:          # 目標速度
            Ltw = 1
            Ltarget = roi.height if roi.y_offset == 0 else -roi.height
        else:                       # ＰＷＭ直接駆動
            Ltw = 0
            Lpwm = roi.height if roi.y_offset == 0 else -roi.height
    else:                           # 右
        if roi.width == 1:          # 目標速度
            Rtw = 1
            Rtarget = roi.height if roi.y_offset == 0 else -roi.height
        else:                       # ＰＷＭ直接駆動
            Rtw = 0
            Rpwm = roi.height if roi.y_offset == 0 else -roi.height
    print("駆動指示：左?Ltw=%d, Ltar=%d, Lpwm=%d, 右?Rtw=%d, Rtar=%d, Rpwm=%d"
                % (Ltw, Ltarget, Lpwm, Rtw, Rtarget, Rpwm))

#==============================================================================
# ４．トピック/param_infoを受信時のコールバック関数：paramCallback(roi)
#   ＜引数 roi：左/右の車輪の制御係数を下記のようにエンコードしている＞
#------------------------------------------------------------------------------
#　・sensor_msgs/RegionOfInterest.msgを使用して（独自型を作成せず）
#　　‐bool do_rectify ：左ならTrue、右ならFalseにエンコード
#　　‐uint32 width    ：目標速度に応じたモーターへの駆動係数
#　　‐uint32 height   ：ＰＩＤ制御の偏差に対する比例制御部の係数
#　　‐uint32 y_offset ：ＰＩＤ制御の偏差に対する積分制御部の係数
#　　‐uint32 x_offset ：ＰＩＤ制御の偏差に対する微分制御部の係数＋ＰＷＭオフセット
#==============================================================================
def paramCallback(roi):
    global Ll, Rl, Lp, Rp, Li, Ri, Ld, Rd, Lo, Ro
    #
    # ＰＩＤ制御の各係数をグローバル変数で指定する
    #
    if roi.do_rectify == True:      # 左
        Ll = roi.width
        Lp = roi.height
        Li = roi.y_offset
        Ld = roi.x_offset % 256
        Lo = roi.x_offset / 256
    else:                           # 右
        Rl = roi.width
        Rp = roi.height
        Ri = roi.y_offset
        Rd = roi.x_offset % 256
        Ro = roi.x_offset / 256
    print("制御係数：左?Ll=%d,Lp=%d,Li=%d,Ld=%d,Lo=%d,右?Rl=%d,Rp=%d,Ri=%d,Rd=%d,Ro=%d"
                % (Ll, Lp, Li, Ld, Lo, Rl, Rp, Ri, Rd, Ro))


#==============================================================================
# ５．メイン処理関数：main()
#------------------------------------------------------------------------------
#　・モーター・ドライバに接続したGPIOポートを出力に設定
#　・モーター・ドライバへの出力をLoにしてモーターをフリーにしておく
#　・モーター駆動用の出力ポートを1kHz周期のデューティー0%でスタート
#　・以降のモーター制御はスレッドで実行する
#　・トピックを受信したときに通知するコールバック関数を設定する
#　・メインスレッドはROSの終了までここで実行
#　・制御スレッドの実行停止を指示して終了を待ち合わせる
#　・終了処理
#==============================================================================
def main():
    global flag,plf,prf,plr,prr

    GPIO.setmode(GPIO.BCM)      # GPIOの指定にBCM番号を使用する
    #
    # モーター・ドライバに接続したGPIOポートを出力に設定
    #
    GPIO.setup(OT_L_F, GPIO.OUT)
    GPIO.setup(OT_L_R, GPIO.OUT)
    GPIO.setup(OT_R_F, GPIO.OUT)
    GPIO.setup(OT_R_R, GPIO.OUT)
    #
    # モーター・ドライバへの出力をLoにしてモーターをフリーにしておく
    #
    GPIO.output(OT_L_F, GPIO.LOW)
    GPIO.output(OT_L_R, GPIO.LOW)
    GPIO.output(OT_R_F, GPIO.LOW)
    GPIO.output(OT_R_R, GPIO.LOW)
    #
    # モーター駆動用の出力ポートを1kHz周期のデューティー0%でスタート
    #
    plf = GPIO.PWM(OT_L_F, PWM_FREQ)
    prf = GPIO.PWM(OT_R_F, PWM_FREQ)
    plr = GPIO.PWM(OT_L_R, PWM_FREQ)
    prr = GPIO.PWM(OT_R_R, PWM_FREQ)
    plf.start(0)
    prf.start(0)
    plr.start(0)
    prr.start(0)
    #
    # 以降のモーター制御はスレッドで実行する
    #
    th = threading.Timer(2, speedControl)
    th.start()

    #--------------------------------------------------------------------------
    # ここからメインスレッドでROSのコードが実行される
    #--------------------------------------------------------------------------
    rospy.init_node('rc_car_pi')
    rospy.loginfo("ROSスレッドを開始しました。")
    #
    # トピックを受信したときに通知するコールバック関数を設定する
    #
    rospy.Subscriber('/cmd_vel', Twist, twistCallback, queue_size=10)
    rospy.Subscriber('/drive_info', ROI, driveCallback, queue_size=10)
    rospy.Subscriber('/param_info', ROI, paramCallback, queue_size=10)
    speedControl.pub = rospy.Publisher('/speed_info', ROI, queue_size=10)
    #
    # メインスレッドはROSの終了までここで実行
    #
    rospy.spin()
    rospy.loginfo("ROSスレッドが終了しました。")
    #
    # 制御スレッドの実行停止を指示して終了を待ち合わせる
    #
    flag = False
    th.join()
    #
    # 終了処理
    #
    rospy.loginfo("アプリケーションを終了します。")
    plf.stop()
    prf.stop()
    plr.stop()
    prr.stop()
    GPIO.cleanup()


#------------------------------------------------------------------------------
# このファイルから起動時のみ実行する
#------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
