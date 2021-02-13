import React from 'react';
import PropTypes from 'prop-types';
import { tfClientToFrame, viewer3d, urdfClient } from '../services/RosService';
import { Grid } from 'ros3d';
import Button from '@material-ui/core/Button';

export class ModelVisualizer extends React.Component {
    viewerDivId = 'urdf';

    constructor(props) {
        super(props);

        this.state = {
            tfClient: null,
            viewer3d: null,
            urdfClient: null
        };
    }

    componentDidMount() {
        let tfClientTmp = tfClientToFrame(this.props.targetFrame, this.props.tfRate);
        let viewer3dTmp = viewer3d(
            this.viewerDivId,
            this.props.width,
            this.props.height,
            this.props.cameraPosition
        );
        viewer3dTmp.addObject(new Grid());

        this.setState(
            {
                tfClient: tfClientTmp,
                viewer3d: viewer3dTmp,
                urdfClient: urdfClient(tfClientTmp, viewer3dTmp, this.props.urdfPath)
            },
            () => { }
        );
    }

    componentWillUnmount() {
        this.state.tfClient.unsubscribe();
        this.state.urdfClient.unsubscribe();
    }

    onMouseClick = () => {
        console.log(this.state.viewer3d.camera.position);
        console.log(this.state.viewer3d.camera);
    };

    render() {
        return (
            <div>
                <div id={this.viewerDivId}></div>
                <Button variant="contained" onClick={this.onMouseClick}>
                    Print Camera
				</Button>
            </div>
        );
    }
}

ModelVisualizer.propTypes = {
    urdfPath: PropTypes.string.isRequired,
    targetFrame: PropTypes.string.isRequired,
    tfRate: PropTypes.number.isRequired,
    width: PropTypes.number.isRequired,
    height: PropTypes.number.isRequired,
    cameraPosition: PropTypes.shape({
        x: PropTypes.number.isRequired,
        y: PropTypes.number.isRequired,
        z: PropTypes.number.isRequired
    })
};
