import os
import boto3

def upload_file_to_s3(file_path, bucket_name, object_name=None):
    """
    Upload a file to an S3 bucket

    :param file_path: Full path to file to upload
    :param bucket_name: Bucket to upload to
    :param object_name: S3 object name. If not specified, just the file name is used
    """
    # Extract the file name from the file path if object_name is not specified
    if object_name is None:
        object_name = os.path.basename(file_path)

    # Upload the file
    s3_client = boto3.client('s3')
    try:
        response = s3_client.upload_file(file_path, bucket_name, object_name)
    except Exception as e:
        print(f"Upload failed: {e}")
    else:
        print(f"File {file_path} uploaded to {bucket_name}/{object_name}")


if __name__ == "__main__":
    file_name = "/home/bhabas/catkin_ws/src/sar_simulation/sar_general/Network_Diagram.pdf"
    bucket_name = "robotlandingproject--deeprl--logs"

    upload_file_to_s3(file_name, bucket_name)