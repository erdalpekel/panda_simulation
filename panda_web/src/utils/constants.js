const panda_simulation = {
    constants: {
        JOINT_STATES_TOPIC: '/joint_states',
        PANDA_EE_PARENT_LINK: 'panda_link8',
        WORLD_LINK: 'world',
        ROBOT_BASE_LINK: 'panda_link0'
    },
    config: {
        tfRate: 25,
        width: 1000,
        height: 600,
        cameraPosition: { x: 3, y: 3, z: 3 }
    }
};

module.exports = {
    panda_simulation
};