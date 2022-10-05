#!/usr/bin/env python3

# ライブラリをインポート
import boto3
import sys


def aws_detect_eyes(image_path):
    """
    AWSのRekognitionを使用して顔の情報を認識
    """
    state = ""

    rekognition = boto3.client("rekognition")

    with open(image_path, "rb") as f:
        response = rekognition.detect_faces(Image={"Bytes": f.read()}, Attributes=["ALL"])
        if len(response["FaceDetails"]) > 0:
            if response["FaceDetails"][0]["EyesOpen"]["Value"]:
                state = "Open"
            else:
                state = "Closed"
        return state


if __name__ == "__main__":
    detect_eyes = aws_detect_eyes("./example_image/example.jpg")
    print(detect_eyes)
