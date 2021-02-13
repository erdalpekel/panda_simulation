import React from 'react';
import PropTypes from 'prop-types';
import { tfClientToFrame } from '../services/RosService';

export class TransformClient extends React.Component {
    float_precision = 3;

    constructor(props) {
        super(props);

        this.state = {
            tfClient: null,
            transform: null
        };
    }

    componentDidMount() {
        this.setState(
            {
                tfClient: tfClientToFrame(this.props.targetFrame, this.props.tfRate)
            },
            () => {
                this.state.tfClient.subscribe(this.props.sourceFrame, this.pandaWorldTransformCallback);
            }
        );
    }

    componentWillUnmount() {
        this.state.tfClient.unsubscribe(this.props.targetFrame);
    }

    pandaWorldTransformCallback = message => {
        this.setState({
            transform: {
                translation: message.translation,
                rotation: message.rotation
            }
        });
    };

    render() {
        // capture variable for lambda functions
        const float_precision = this.float_precision;

        return (
            <div>
                {this.state.transform ? (
                    <div>
                        <h2>
                            {this.props.sourceFrame} to {this.props.targetFrame} transform:
						</h2>

                        <h3>translation</h3>
                        <span>x: {this.state.transform.translation.x.toFixed(float_precision)} </span>
                        <span>y: {this.state.transform.translation.y.toFixed(float_precision)} </span>
                        <span>z: {this.state.transform.translation.z.toFixed(float_precision)} </span>

                        <h3>rotation</h3>
                        <span>x: {this.state.transform.rotation.x.toFixed(float_precision)} </span>
                        <span>y: {this.state.transform.rotation.y.toFixed(float_precision)} </span>
                        <span>z: {this.state.transform.rotation.z.toFixed(float_precision)} </span>
                        <span>w: {this.state.transform.rotation.w.toFixed(float_precision)} </span>
                    </div>
                ) : null}
            </div>
        );
    }
}

TransformClient.propTypes = {
    targetFrame: PropTypes.string.isRequired,
    sourceFrame: PropTypes.string.isRequired,
    tfRate: PropTypes.number.isRequired
};
