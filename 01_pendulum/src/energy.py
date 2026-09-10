import mujoco


def get_energy(model, data):
    # data.energy = [potential, kinetic]
    potential_energy = data.energy[0]
    kinetic_energy = data.energy[1]

    total_energy = kinetic_energy + potential_energy

    return kinetic_energy, potential_energy, total_energy
