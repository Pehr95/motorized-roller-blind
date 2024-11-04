Import("env")
import subprocess

def after_upload(source, target, env):
    # Run the Serial Monitor after uploading
    subprocess.call("pio device monitor", shell=True)

# Attach the after_upload function to run after the upload action
env.AddPostAction("upload", after_upload)
