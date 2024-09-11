OV_PATH = "omniverse://kit-test-content.ov.nvidia.com/Projects/DemoContent/DoNotDistribute/Physics_Release/"
S3_BUCKET = "omniverse-content-staging"
S3_REGION = "us-west-2"
S3_PATH = "DoNotDelete/PhysicsDemoAssets/"


def get_assets_path(force_ov_path=False, force_s3_path=False):
    import omni.kit.app
    version = omni.kit.app.get_app().get_kit_version()
    mm_version = ".".join(version.split("+")[0].split(".")[:2])

    if force_ov_path:
        demo_devel_mode = False
    elif force_s3_path:
        demo_devel_mode = True
    else:
        import carb.settings
        demo_devel_mode = carb.settings.get_settings().get("physics/demoDevelopmentMode")

    server_path = OV_PATH if demo_devel_mode else get_s3_web_path()
    asset_path = f"{server_path}{mm_version}/"

    return asset_path


def get_s3_web_path():
    return f"https://{S3_BUCKET}.s3.{S3_REGION}.amazonaws.com/{S3_PATH}"


def get_s3_upload_path():
    return f"s3://{S3_BUCKET}/{S3_PATH}"
