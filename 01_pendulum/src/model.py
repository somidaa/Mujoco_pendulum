import mujoco


def load_model(xml_path):
    model = mujoco.MjModel.from_xml_path(xml_path)
    return model


def create_data(model):
    data = mujoco.MjData(model)
    return data
