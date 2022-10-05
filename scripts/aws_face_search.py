#!/usr/bin/env python3

# ライブラリのインポート
import boto3
from botocore.exceptions import ClientError
import time
import random
import os

# 自作のライブラリをインポート
from aws_face_collection import AWSFaceCollection


class AWSFaceSearch():
    def __init__(self):
        """ 初期化処理 """
        # AWSを使う準備
        self.rekognition = boto3.client(service_name="rekognition")

        # コレクション情報を取得
        try:
            self.collections_info = self.rekognition.list_collections()
        except ClientError as e:
            self.print_error(e)

    def search_collection(self, user_id, path):
        """ 顔認証を行う関数 """
        # 指定されたパスの画像を開く
        with open(path, "rb") as f:
            try:
                # 顔認証を実行
                return self.rekognition.search_faces_by_image(CollectionId=user_id, Image={"Bytes": f.read()})
            except ClientError as e:
                self.print_error(e)
                return None

    def print_error(self, error):
        """ 例外処理時のログ表示用 """
        if error.response["Error"]["Code"] == "InvalidParameterException":
            print("画像中に顔が写ってません。")
        elif error.response["Error"]["Code"] == "ProvisionedThroughputExceededException":
            print("AWSが混み合っていますので、しばらくお待ちください。")
            time.sleep(int(random.uniform(0, 5)))
        else:
            print("エラーが発生しました。")


if __name__ == "__main__":
    # インスタンスを作成
    afc = AWSFaceCollection()
    print("-------------------------------")
    collection_id = input("IDを入力してください: ")
    # IDを登録する
    if afc.register_collection_id(collection_id):
        # 顔画像のパスを指定
        image_path = "./example_image/example.jpg"
        # 指定されたパスが存在するか確認する
        if os.path.exists(image_path):
            # IDに顔画像を登録する
            if afc.register_face_image(image_path):
                print("顔画像の登録が完了しました。")
                print("-------------------------------")
                print("ID: 「{}」に登録されている顔画像数: {}".format(collection_id, afc.face_counter))
                print("-------------------------------")
            else:
                print("顔画像の登録ができませんでした。")
            # インスタンスを作成
            afs = AWSFaceSearch()
            result_data = afs.search_collection(collection_id, image_path)["FaceMatches"]
            if len(result_data) > 0:
                print("登録されている顔を検出しました。")
                print("信頼度: {}％".format(result_data[0]["Face"]["Confidence"]))
                print("類似度: {}％".format(result_data[0]["Similarity"]))
            else:
                print("登録されていない顔を検出しました。")
            print("-------------------------------")
        # 登録情報を削除
        afc.delete_collection_id()
        print("ID: 「{}」の登録を削除しました。".format(collection_id))
    else:
        print("ID: 「{}」 は登録済みです。".format(collection_id))
        print("-------------------------------")
