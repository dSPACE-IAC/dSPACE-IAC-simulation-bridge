$env:AWS_ACCESS_KEY_ID = ""
$env:AWS_SECRET_ACCESS_KEY = ""

aws ecr get-login-password --region eu-central-1 | docker login --username AWS --password-stdin https://722180079256.dkr.ecr.eu-central-1.amazonaws.com
