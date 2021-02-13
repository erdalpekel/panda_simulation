import React from 'react';
import PropTypes from 'prop-types';
import { tfClientToFrame, viewer3d, markerClient } from '../services/RosService';

export class SimpleMarker extends React.Component {
    viewerDivId = 'marker';

    constructor(props) {
        super(props);

        this.state = {
            tfClient: null,
            viewer3d: null,
            markerClient: null
        };
    }

    componentDidMount() {
        let tfClientTmp = tfClientToFrame(this.props.targetFrame, this.props.tfRate);
        let viewer3dTmp = viewer3d(this.viewerDivId, this.props.width, this.props.height);

        this.setState(
            {
                tfClient: tfClientTmp,
                viewer3d: viewer3dTmp,
                markerClient: markerClient(tfClientTmp, this.props.markerTopic, viewer3dTmp)
            },
            () => { }
        );
    }

    componentWillUnmount() {
        this.state.tfClient.unsubscribe();
        this.state.markerClient.unsubscribe();
    }

    render() {
        return <div id={this.viewerDivId}></div>;
    }
}

SimpleMarker.propTypes = {
    markerTopic: PropTypes.string.isRequired,
    targetFrame: PropTypes.string.isRequired,
    tfRate: PropTypes.number.isRequired,
    width: PropTypes.number.isRequired,
    height: PropTypes.number.isRequired
};
