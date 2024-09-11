# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import numpy as np
import torch
import warp as wp
from omni.replicator.core import AnnotatorRegistry, BackendDispatch, Writer, WriterRegistry
from omni.replicator.isaac.scripts.writers.pytorch_listener import PytorchListener

__version__ = "0.0.1"


class PytorchWriter(Writer):
    """A custom writer that uses omni.replicator API to retrieve RGB data via render products
        and formats them as tensor batches. The writer takes a PytorchListener which is able
        to retrieve pytorch tensors for the user directly after each writer call.

    Args:
        listener (PytorchListener): A PytorchListener that is sent pytorch batch tensors at each write() call.
        output_dir (str): directory in which rgb data will be saved in PNG format by the backend dispatch.
                          If not specified, the writer will not write rgb data as png and only ping the
                          listener with batched tensors.
        device (str): device in which the pytorch tensor data will reside. Can be "cpu", "cuda", or any
                      other format that pytorch supports for devices. Default is "cuda".
    """

    def __init__(
        self, listener: PytorchListener, output_dir: str = None, tiled_sensor: bool = False, device: str = "cuda"
    ):
        # If output directory is specified, writer will write annotated data to the given directory
        if output_dir:
            self.backend = BackendDispatch({"paths": {"out_dir": output_dir}})
            self._backend = self.backend
            self._output_dir = self.backend.output_dir
        else:
            self._output_dir = None
        self._frame_id = 0

        if tiled_sensor:
            self.annotators = [AnnotatorRegistry.get_annotator("RtxSensorGpu", device="cuda", do_array_copy=False)]
        else:
            self.annotators = [AnnotatorRegistry.get_annotator("LdrColor", device="cuda", do_array_copy=False)]

        self.listener = listener
        self.device = device
        self.version = __version__

    def write(self, data: dict) -> None:
        """Sends data captured by the attached render products to the PytorchListener and will write data to
        the output directory if specified during initialization.

        Args:
            data (dict): Data to be pinged to the listener and written to the output directory if specified.
        """
        # breakpoint()
        for annotator in data.keys():
            if annotator.startswith("rp"):
                rp_info = data[annotator]

        if self._output_dir:
            # Write RGB data to output directory as png
            self._write_rgb(data, rp_info)
        pytorch_rgb = self._convert_to_pytorch(data, rp_info).to(self.device)
        self.listener.write_data({"pytorch_rgb": pytorch_rgb, "device": self.device})
        self._frame_id += 1

    @carb.profiler.profile
    def _write_rgb(self, data: dict, rp_info: dict) -> None:
        for annotator in data.keys():
            if annotator.startswith("LdrColor"):
                render_product_name = annotator.split("-")[-1]
                file_path = f"rgb_{self._frame_id}_{render_product_name}.png"
                img_data = data[annotator]
                if isinstance(img_data, wp.types.array):
                    img_data = img_data.numpy()
                self._backend.write_image(file_path, img_data)
            elif annotator.startswith("RtxSensor"):
                width, height = rp_info["resolution"][0], rp_info["resolution"][1]
                file_path = f"rgb_{self._frame_id}.png"
                img_data = data[annotator].reshape(height, width, -1)
                self._backend.write_image(file_path, img_data)

    @carb.profiler.profile
    def _convert_to_pytorch(self, data: dict, rp_info: dict) -> torch.Tensor:
        if data is None:
            raise Exception("Data is Null")
        # breakpoint()
        data_tensors = []
        for annotator in data.keys():
            if annotator.startswith("LdrColor"):
                data_tensors.append(wp.to_torch(data[annotator]).unsqueeze(0))
            elif annotator.startswith("RtxSensor"):
                breakpoint()
                width, height = rp_info["resolution"][0], rp_info["resolution"][1]
                img_data = data[annotator]
                data_tensors.append(wp.to_torch(img_data).reshape(height, width, -1).unsqueeze(0))
            elif annotator.startswith("distance"):
                width, height = rp_info["resolution"][0], rp_info["resolution"][1]
                data_tensors.append(wp.to_torch(data[annotator]).reshape(height, width, -1).unsqueeze(0))

        # Move all tensors to the same device for concatenation
        device = "cuda:0" if self.device == "cuda" else self.device
        data_tensors = [t.to(device) for t in data_tensors]

        data_tensor = torch.cat(data_tensors, dim=0)
        return data_tensor


WriterRegistry.register(PytorchWriter)
# Adding to default writers for Replicator telemetry tracking
WriterRegistry._default_writers.append(
    "PytorchWriter"
) if "PytorchWriter" not in WriterRegistry._default_writers else None
