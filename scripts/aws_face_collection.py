#!/usr/bin/env python3

# ライブラリのインポート
import boto3
import os
from botocore.exceptions import ClientError
import time
import random


class AWSFaceCollection():
    def __init__(self):
        """ 初期化処理 """
        # AWSを使う準備
        self.rekognition = boto3.client(service_name="rekognition")
        # 変数の初期化
        self.collection_id = ""
        self.face_counter = 0

        # コレクション情報を取得
        try:
            self.collections_info = self.rekognition.list_collections()
        except ClientError as e:
            self.print_error(e)

    def delete_collection_id(self):
        """ IDを削除する関数 """
        try:
            # コレクション情報を再取得
            self.collections_info = self.rekognition.list_collections()
            # IDが登録されている場合は削除
            if self.collection_id != "":
                self.rekognition.delete_collection(CollectionId=self.collection_id)
                self.face_counter = 0
        except ClientError as e:
            self.print_error(e)

    def check_collection_id(self, user_id):
        """ IDが登録されているか確認する関数 """
        try:
            # コレクション情報を再取得
            self.collections_info = self.rekognition.list_collections()
            # IDがコレクションに含まれるか確認
            return user_id in self.collections_info["CollectionIds"]
        except ClientError as e:
            self.print_error(e)
            return None

    def register_collection_id(self, user_id):
        """ IDを登録する関数 """
        try:
            # コレクション情報を再取得
            self.collections_info = self.rekognition.list_collections()
            # IDが登録されていないことを確認して登録処理を実行
            if user_id in self.collections_info["CollectionIds"]:
                return False
            else:
                # コレクションの登録
                self.rekognition.create_collection(CollectionId=user_id)
                # IDを変数に保持しておく
                self.collection_id = user_id
                return True
        except ClientError as e:
            self.print_error(e)
            return None

    def register_face_image(self, path):
        """ IDに顔画像を登録する関数 """
        try:
            # 指定されたパスの画像を開く
            with open(path, "rb") as f:
                # AWSに顔画像データを登録する
                result_data = self.rekognition.index_faces(CollectionId=self.collection_id, Image={"Bytes": f.read()})
                # 正常に登録できた場合
                if len(result_data["FaceRecords"]) > 0:
                    self.face_counter += 1
                    return True
                else:
                    return False
        except ClientError as e:
            self.print_error(e)
            return False

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
        # 終了時にIDを自動的に削除
        afc.delete_collection_id()
        print("ID: 「{}」の登録を削除しました。".format(collection_id))
    else:
        print("ID: 「{}」 は登録済みです。".format(collection_id))
        print("-------------------------------")
