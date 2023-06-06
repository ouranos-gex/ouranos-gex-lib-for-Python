# Copyright © 2022 Digital Agency. All rights reserved.
# NOTE: 出力レベルは未使用のものも列記
from logging import getLogger, INFO, DEBUG, WARNING, ERROR
import logging
import logging.handlers
import os


# ロガーの名前
LOGGER_NAME = "spatial_id"

# 実行位置の絶対パスの取得
ROOT_PATH = os.path.dirname(__file__)

# ログ出力先の相対パス
LOG_FILE_PATH = "./log/"

# ログファイル名
LOG_FILE_NAME = "spatial.log"

# ロガーへの設定
LOGGER_PATH = os.path.join(ROOT_PATH, LOG_FILE_PATH, LOG_FILE_NAME)

try:
    # ログディレクトリがない場合は作成する
    if not os.path.exists(os.path.join(ROOT_PATH, LOG_FILE_PATH)):
        os.mkdir(os.path.join(ROOT_PATH, LOG_FILE_PATH))
except Exception:
    # 作成に失敗しても何もしない
    pass

# メッセージ出力レベル
MESSAGE_LEVEL = DEBUG


class logger():
    logger = getLogger(LOGGER_NAME)
    logger.setLevel(MESSAGE_LEVEL)
    # 出力時間 - メッセージレベル - モジュールファイル名 - API名 - 出力メッセージ
    formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(name)s - %(funcName)s - %(message)s')
    try:
        rotate = logging.handlers.RotatingFileHandler(
            LOGGER_PATH,
            encoding='utf-8',
            maxBytes=1000000,
            backupCount=3
        )
        # 書式の設定
        rotate.setFormatter(formatter)
        # FileHandlerの追加
        logger.addHandler(rotate)
    except Exception:
        # 作成に失敗しても何もしない
        pass

