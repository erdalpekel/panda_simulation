import React from 'react';
import PropTypes from 'prop-types';
import { JointStatesListener } from '../services/RosService';
import Table from '@material-ui/core/Table';
import TableBody from '@material-ui/core/TableBody';
import TableCell from '@material-ui/core/TableCell';
import TableContainer from '@material-ui/core/TableContainer';
import TableHead from '@material-ui/core/TableHead';
import TableRow from '@material-ui/core/TableRow';
import Paper from '@material-ui/core/Paper';

export class JointStates extends React.Component {
    float_precision = 3;

    constructor(props) {
        super(props);

        this.state = {
            jointStatesListener: null,
            jointStates: null
        };
    }

    componentDidMount() {
        this.setState(
            {
                jointStatesListener: JointStatesListener(this.props.topic)
            },
            () => {
                this.state.jointStatesListener.subscribe(this.jointStatesCallback);
            }
        );
    }

    componentWillUnmount() {
        this.state.jointStatesListener.unsubscribe();
    }

    jointStatesCallback = message => {
        this.setState({
            jointStates: {
                name: message.name,
                position: message.position,
                effort: message.effort,
                velocity: message.velocity,
                frame_id: message.header.frame_id
            }
        });
    };

    render() {
        // capture variable for lambda functions
        const float_precision = this.float_precision;

        const classes = {
            table: {
                minWidth: 650
            }
        };

        return (
            <div>
                {this.state.jointStates ? (
                    <TableContainer component={Paper}>
                        <Table className={classes.table} size="small" aria-label="a dense table">
                            <TableHead>
                                <TableRow>
                                    <TableCell>Topic: {this.props.topic}</TableCell>
                                    {this.state.jointStates.name.map(function (value, index) {
                                        return <TableCell align="right">{value}</TableCell>;
                                    })}
                                </TableRow>
                            </TableHead>
                            <TableBody>
                                <TableRow key={'position'}>
                                    <TableCell component="th" scope="row">
                                        {'position'}
                                    </TableCell>
                                    {this.state.jointStates.position.map(function (value, index) {
                                        return <TableCell align="right">{value.toFixed(float_precision)}</TableCell>;
                                    })}
                                </TableRow>
                                <TableRow key={'velocity'}>
                                    <TableCell component="th" scope="row">
                                        {'velocity'}
                                    </TableCell>
                                    {this.state.jointStates.velocity.map(function (value, index) {
                                        return <TableCell align="right">{value.toFixed(float_precision)}</TableCell>;
                                    })}
                                </TableRow>
                                <TableRow key={'effort'}>
                                    <TableCell component="th" scope="row">
                                        {'effort'}
                                    </TableCell>
                                    {this.state.jointStates.effort.map(function (value, index) {
                                        return <TableCell align="right">{value.toFixed(float_precision)}</TableCell>;
                                    })}
                                </TableRow>
                            </TableBody>
                        </Table>
                    </TableContainer>
                ) : null}
            </div>
        );
    }
}

JointStates.propTypes = {
    topic: PropTypes.string.isRequired
};
