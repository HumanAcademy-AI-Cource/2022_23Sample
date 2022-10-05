#!/usr/bin/env python3

# ライブラリをインポート
import rospy
import roslib.packages
import cv2
import os
import signal
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import *
from std_msgs.msg import *


# 自作のライブラリをインポート
from aws_face_collection import AWSFaceCollection
from aws_face_search import AWSFaceSearch


class DetectSpecifiedFace():
    def __init__(self):
        """ 初期化処理 """
        # サブスクライバーを定義
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # パブリッシャーを定義
        self.led_pub = rospy.Publisher("/led", ColorRGBA, queue_size=1)
        # 画像を保持する変数
        self.image = None

        # 自身のROSパッケージの場所を取得
        self.pkg_dir = roslib.packages.get_pkg_dir("myface_recognition")
        # 画像の保存先ディレクトリのパスを定義
        self.image_dir = self.pkg_dir + "/scripts/images/"
        # ディレクトリがない場合は新しく作成
        if not os.path.isdir(self.image_dir):
            os.mkdir(self.image_dir)

        # カメラ画像のパス
        self.camera_image_path = self.image_dir + "camera.jpg"

        # 自作ライブラリの準備
        self.afc = AWSFaceCollection()
        self.afs = AWSFaceSearch()

        # Ctrl-Cが実行されたときの処理用
        signal.signal(signal.SIGINT, self.signal_handler)

    def image_callback(self, data):
        """ 画像のコールバック関数 """
        try:
            self.image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def image_save(self):
        """ 画像を保存する関数 """
        cv2.imwrite(self.camera_image_path, self.image)
        print("カメラ画像を保存しました。")

    def search_collection(self, collection_id):
        """ 顔認証を行う関数 """
        # 顔認証を実行
        result_data = self.afs.search_collection(collection_id, self.camera_image_path)
        # 顔認証が正常に完了した場合
        color = ColorRGBA()
        if result_data != None:
            # マッチした顔が見つかった場合
            if len(result_data["FaceMatches"]) > 0:
                print("-------------------------------")
                print("登録されている顔を検出しました。")
                print("-------------------------------")
                print("信頼度: {}％".format(result_data["FaceMatches"][0]["Face"]["Confidence"]))
                print("類似度: {}％".format(result_data["FaceMatches"][0]["Similarity"]))
                color.r = 0
                color.g = 255.0
                color.b = 0
            else:
                print("-------------------------------")
                print("登録されていない顔を検出しました。")
                color.r = 255.0
                color.g = 0
                color.b = 0
            print("-------------------------------")
        else:
            color.r = 255.0
            color.g = 0
            color.b = 0
        self.led_pub.publish(color)

    def signal_handler(self, signal, frame):
        """ Ctrl-Cが実行されたときの処理用の関数 """
        sys.exit(0)

    def local_id_list(self):
        """ ローカルに保存されているディレクトリ（ID）を一覧表示 """
        dirs = os.listdir(self.image_dir)
        print("-----------------------------------------------")
        print("このコンピュータ内に保存されているID")
        dir_counter = 0
        for d in dirs:
            if os.path.isdir(os.path.join(self.image_dir, d)):
                print("○ {}".format(d))
                dir_counter += 1
        if dir_counter == 0:
            print("-> 保存されていません。")

    def main(self):
        """ メインで実行する関数 """
        # しばらく待つ
        rospy.sleep(2.0)
        while True:
            # IDのリストを表示
            self.local_id_list()
            print("-----------------------------------------------")
            collection_id = input("登録した自身のIDを入力してください: ")
            # ローカルにディレクトリ（ID）存在するか確認
            collection_dir = self.image_dir + collection_id
            if not os.path.isdir(collection_dir):
                print("-----------------------------------------------")
                print("このコンピュータに保存されているIDを入力してください。")
                rospy.sleep(2.0)
                continue
            # IDが存在するか確認
            if self.afc.check_collection_id(collection_id):
                # 登録されていた場合は、インスタンスに設定を反映
                self.afc.collection_id = collection_id
                break
            else:
                print("ID: 「{}」 はAWS上に登録されていません。".format(collection_id))
                rospy.sleep(2.0)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.image is not None:
                input("Enterキーを押すと顔認識を開始します:")
                # 画像を保存
                self.image_save()
                # 顔認証処理を開始
                self.search_collection(self.afc.collection_id)
            else:
                print("画像が未取得です。")
            rate.sleep()


if __name__ == "__main__":
    # ノードを初期化
    rospy.init_node("face_search")
    # クラスのインスタンスを作成し、メイン関数を実行
    DetectSpecifiedFace().main()
