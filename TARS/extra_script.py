# Simple script to add delay between upload and start of unit tests
# Necessary on some hosts to allow the target Serial port to catch-up

import time
Import("env")


def after_upload(source, target, env):
    print("Delay while uploading...")
    time.sleep(1)


env.AddPostAction("upload", after_upload)
