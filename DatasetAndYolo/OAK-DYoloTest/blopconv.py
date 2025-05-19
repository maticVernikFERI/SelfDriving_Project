import blobconverter

# Convert model and save as .blob
blob_path = blobconverter.from_openvino(
    xml="best.xml",  # Your OpenVINO IR model file
    bin="best.bin",  # Corresponding weights file
    data_type="FP16",
    shaves=6  # Adjust based on your OAK-D hardware
)

print(f"Blob model saved at: {blob_path}")