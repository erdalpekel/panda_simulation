import React from 'react';
import { JointStates } from '../components/JointStates';
import { TransformClient } from '../components/TransformClient';
import { ModelVisualizer } from '../components/ModelVisualizer';
import { panda_simulation } from '../utils/constants';

export default class DashboardView extends React.Component {
    constructor(props) {
        super(props);

        this.state = {};
    }

    render() {
        return (
            <div>
                <JointStates topic={panda_simulation.constants.JOINT_STATES_TOPIC} />
                <TransformClient
                    targetFrame={panda_simulation.constants.WORLD_LINK}
                    sourceFrame={panda_simulation.constants.PANDA_EE_PARENT_LINK}
                    tfRate={10}
                />
                <ModelVisualizer
                    urdfPath={
                        'http://' + process.env.REACT_APP_FILE_SERVER_URL + ':' + process.env.REACT_APP_FILE_SERVER_PORT
                    }
                    targetFrame={panda_simulation.constants.ROBOT_BASE_LINK}
                    tfRate={panda_simulation.config.tfRate}
                    width={panda_simulation.config.width}
                    height={panda_simulation.config.height}
                    cameraPosition={panda_simulation.config.cameraPosition}
                />
            </div>
        );
    }
}
