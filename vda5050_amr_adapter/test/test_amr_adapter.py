# BSD 3-Clause License
#
# Copyright 2026 Eurecat
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Eurecat nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import asyncio
import math
from unittest.mock import ANY
from unittest.mock import AsyncMock
from unittest.mock import MagicMock

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
from rclpy.action import CancelResponse
from sensor_msgs.msg import BatteryState as ROSBatteryState
from tf2_ros import TransformException
import tf_transformations
from vda5050_amr_adapter import amr_adapter
from vda5050_amr_adapter.amr_adapter import AMRAdapter
from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction
from vda5050_connector.srv import GetState
from vda5050_connector.srv import SupportedActions
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import ActionParameter
from vda5050_msgs.msg import AGVAction
from vda5050_msgs.msg import AGVPosition
from vda5050_msgs.msg import CurrentAction
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import NodePosition
from vda5050_msgs.msg import Velocity

NAMESPACE = '/fleet'
BASE_INTERFACE = f'{NAMESPACE}/robots/robot_1'


@pytest.fixture
def ros_params():
    """Parameter overrides consumed by read_str_parameter; empty means defaults."""
    return {}


@pytest.fixture
def adapter(mocker, ros_params):
    # The ROS node itself and the interfaces it creates are mocked, so the
    # adapter can be exercised without a running ROS graph.
    mocker.patch.object(amr_adapter.Node, '__init__', return_value=None)
    mocker.patch.object(amr_adapter.Node, 'get_logger')
    mocker.patch.object(amr_adapter.Node, 'get_namespace', return_value=NAMESPACE)
    mocker.patch.object(amr_adapter.Node, 'create_service')
    mocker.patch.object(amr_adapter.Node, 'create_publisher')
    mocker.patch.object(amr_adapter.Node, 'create_subscription')
    get_clock = mocker.patch.object(amr_adapter.Node, 'get_clock')
    get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=10, nanosec=5)
    mocker.patch.object(amr_adapter, 'ActionServer')
    mocker.patch.object(amr_adapter, 'ActionClient')
    mocker.patch.object(amr_adapter, 'Buffer')
    mocker.patch.object(amr_adapter, 'TransformListener')
    mocker.patch.object(
        amr_adapter,
        'read_str_parameter',
        side_effect=lambda node, name, default: ros_params.get(name, default),
    )
    return AMRAdapter()


def _transform(x, y, yaw):
    transform = TransformStamped()
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    return transform


def _yaw(orientation):
    return tf_transformations.euler_from_quaternion(
        (orientation.x, orientation.y, orientation.z, orientation.w)
    )[2]


def _vda_goal_handle(action_type, params=None):
    goal_handle = MagicMock()
    goal_handle.request.action = Action(
        action_id='action-1',
        action_type=action_type,
        action_description='test action',
        blocking_type='HARD',
        action_parameters=[
            ActionParameter(key=key, value=str(value))
            for key, value in (params or {}).items()
        ],
    )
    return goal_handle


def _vda_node(x=2.0, y=0.95, theta=-0.66):
    return VDANode(
        node_id='node1',
        sequence_id=0,
        released=True,
        node_position=NodePosition(x=x, y=y, theta=theta, map_id='map'),
        actions=[],
    )


def _nav_goal_handle(node, is_cancel_requested=False):
    goal_handle = MagicMock()
    goal_handle.request.node = node
    goal_handle.is_cancel_requested = is_cancel_requested
    return goal_handle


def _setup_nav2_goal(adapter, accepted=True, status=GoalStatus.STATUS_SUCCEEDED):
    nav_handle = MagicMock()
    nav_handle.accepted = accepted
    nav_handle.get_result_async = AsyncMock(return_value=MagicMock(status=status))
    adapter._nav_to_pose_amr_action_client.send_goal_async = AsyncMock(
        return_value=nav_handle
    )
    return nav_handle


# Construction and configuration


def test_read_parameters_uses_defaults(adapter):
    assert adapter._robot_name == 'robot_1'
    assert adapter._manufacturer_name == 'robots'
    assert adapter._serial_number == 'robot_1'
    assert adapter._map_frame == 'map'
    assert adapter._vda_map_id == 'map'
    assert adapter._robot_base_frame == 'base_link'
    assert adapter._get_state_svc_srv == 'adapter/get_state'
    assert adapter._vda_action_act_srv == 'adapter/vda_action'
    assert adapter._nav_to_node_act_srv == 'adapter/nav_to_node'
    assert adapter._supported_actions_svc_srv == 'adapter/supported_actions'


def test_read_parameters_applies_overrides(adapter, ros_params):
    ros_params.update(
        {
            'robot_name': 'AMR_2',
            'manufacturer_name': 'OSRF',
            'serial_number': 'SN-2',
            'map_frame': 'factory_map',
            'robot_base_frame': 'base_footprint',
            'nav_to_node_act_name': 'custom/nav_to_node',
        }
    )

    adapter.read_parameters()

    assert adapter._robot_name == 'AMR_2'
    assert adapter._manufacturer_name == 'OSRF'
    assert adapter._serial_number == 'SN-2'
    assert adapter._map_frame == 'factory_map'
    assert adapter._robot_base_frame == 'base_footprint'
    assert adapter._nav_to_node_act_srv == 'custom/nav_to_node'


def test_vda_map_id_defaults_to_map_frame(adapter, ros_params):
    ros_params['map_frame'] = 'warehouse_map'

    adapter.read_parameters()

    assert adapter._vda_map_id == 'warehouse_map'


def test_on_configure_creates_namespaced_services(adapter):
    adapter.create_service.assert_any_call(
        srv_type=GetState,
        srv_name=f'{BASE_INTERFACE}/adapter/get_state',
        callback=adapter.get_state_callback,
    )
    adapter.create_service.assert_any_call(
        SupportedActions,
        f'{BASE_INTERFACE}/adapter/supported_actions',
        adapter.supported_actions_callback,
    )


def test_on_configure_uses_custom_service_names(adapter, ros_params):
    ros_params.update(
        {
            'robot_name': 'AMR_2',
            'manufacturer_name': 'OSRF',
            'get_state_svc_name': 'state/get',
        }
    )

    adapter.on_configure()

    adapter.create_service.assert_any_call(
        srv_type=GetState,
        srv_name=f'{NAMESPACE}/OSRF/AMR_2/state/get',
        callback=adapter.get_state_callback,
    )


def test_on_configure_creates_action_servers(adapter):
    amr_adapter.ActionServer.assert_any_call(
        node=adapter,
        action_type=NavigateToNode,
        action_name=f'{BASE_INTERFACE}/adapter/nav_to_node',
        execute_callback=adapter.navigate_to_node_callback,
        callback_group=ANY,
        cancel_callback=adapter.cancel_callback,
    )
    amr_adapter.ActionServer.assert_any_call(
        node=adapter,
        action_type=ProcessVDAAction,
        action_name=f'{BASE_INTERFACE}/adapter/vda_action',
        execute_callback=adapter.process_vda_action_callback,
    )


def test_on_configure_creates_nav2_client_and_ros_interfaces(adapter):
    amr_adapter.ActionClient.assert_called_once_with(
        node=adapter,
        action_type=amr_adapter.NavigateToPose,
        action_name='/navigate_to_pose',
        callback_group=ANY,
    )
    adapter.create_publisher.assert_called_once_with(
        PoseWithCovarianceStamped, '/initialpose', 10
    )
    adapter.create_subscription.assert_any_call(
        Odometry, '/odom', adapter.odom_callback, 10
    )
    adapter.create_subscription.assert_any_call(
        ROSBatteryState, '/battery_state', adapter.battery_callback, 10
    )
    amr_adapter.TransformListener.assert_called_once_with(adapter._tf_buffer, adapter)


# Sensor callbacks


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 3])
def test_odom_callback_updates_pose_and_velocity(adapter, yaw):
    adapter._tf_buffer.lookup_transform.return_value = _transform(1.5, -2.0, yaw)
    odom = Odometry()
    odom.twist.twist.linear.x = 0.4
    odom.twist.twist.linear.y = 0.1
    odom.twist.twist.angular.z = 0.2

    adapter.odom_callback(odom)

    assert adapter._tf_buffer.lookup_transform.call_args.args[:2] == ('map', 'base_link')
    assert adapter._agv_position.position_initialized is True
    assert adapter._agv_position.x == pytest.approx(1.5)
    assert adapter._agv_position.y == pytest.approx(-2.0)
    assert adapter._agv_position.theta == pytest.approx(yaw)
    assert adapter._agv_position.map_id == 'map'
    assert adapter._velocity.vx == pytest.approx(0.4)
    assert adapter._velocity.vy == pytest.approx(0.1)
    assert adapter._velocity.omega == pytest.approx(0.2)


def test_odom_callback_flags_position_uninitialized_on_tf_error(adapter):
    adapter._tf_buffer.lookup_transform.side_effect = TransformException('no transform')
    adapter._agv_position.position_initialized = True
    adapter.logger.info.reset_mock()

    adapter.odom_callback(Odometry())

    assert adapter._agv_position.position_initialized is False
    adapter.logger.info.assert_called_once()


@pytest.mark.parametrize(
    'current, expected_charging',
    [(-1.0, False), (0.0, False), (1.5, True)],
)
def test_battery_callback_maps_battery_state(adapter, current, expected_charging):
    battery_msg = ROSBatteryState(percentage=0.75, voltage=24.5, current=current)

    adapter.battery_callback(battery_msg)

    assert adapter._battery.battery_charge == pytest.approx(75.0)
    assert adapter._battery.battery_voltage == pytest.approx(24.5)
    assert adapter._battery.charging is expected_charging


# Services


def test_get_state_callback_reports_current_state(adapter):
    adapter._agv_position = AGVPosition(
        x=1.0, y=2.0, theta=0.5, map_id='map', position_initialized=True
    )
    adapter._velocity = Velocity(vx=0.1, vy=0.0, omega=0.2)
    adapter._battery.battery_charge = 80.0
    adapter._driving = True

    response = adapter.get_state_callback(None, GetState.Response())

    state = response.state
    assert state.agv_position.x == pytest.approx(1.0)
    assert state.agv_position.y == pytest.approx(2.0)
    assert state.agv_position.position_initialized is True
    assert state.velocity.vx == pytest.approx(0.1)
    assert state.battery_state.battery_charge == pytest.approx(80.0)
    assert state.driving is True


def test_supported_actions_callback_advertises_init_position(adapter):
    response = adapter.supported_actions_callback(None, SupportedActions.Response())

    assert len(response.agv_actions) == 1
    init_position = response.agv_actions[0]
    assert init_position.action_type == 'initPosition'
    assert init_position.action_scopes == [AGVAction.INSTANT]
    assert [param.key for param in init_position.action_parameters] == ['x', 'y', 'theta']
    for param in init_position.action_parameters:
        assert param.value_data_type == param.FLOAT
        assert param.is_optional is False


# VDA actions


def test_init_position_action_publishes_initial_pose(adapter):
    goal_handle = _vda_goal_handle(
        'initPosition', {'x': 1.0, 'y': -2.5, 'theta': math.pi / 4}
    )

    result = adapter.process_vda_action_callback(goal_handle)

    adapter._init_pose_pub.publish.assert_called_once()
    pose_msg = adapter._init_pose_pub.publish.call_args.args[0]
    assert pose_msg.header.frame_id == 'map'
    assert pose_msg.pose.pose.position.x == pytest.approx(1.0)
    assert pose_msg.pose.pose.position.y == pytest.approx(-2.5)
    assert _yaw(pose_msg.pose.pose.orientation) == pytest.approx(math.pi / 4)

    assert result.result.action_id == 'action-1'
    assert result.result.action_status == CurrentAction.FINISHED
    goal_handle.succeed.assert_called_once()


def test_unsupported_action_publishes_feedback_sequence(mocker, adapter):
    sleep = mocker.patch.object(amr_adapter.time, 'sleep')
    goal_handle = _vda_goal_handle('pickLoad')
    # The feedback message is reused by the adapter, so record each status on publish.
    statuses = []
    goal_handle.publish_feedback.side_effect = (
        lambda msg: statuses.append(msg.current_action.action_status)
    )

    result = adapter.process_vda_action_callback(goal_handle)

    assert statuses == [
        CurrentAction.INITIALIZING,
        CurrentAction.RUNNING,
        CurrentAction.PAUSED,
        CurrentAction.RUNNING,
        CurrentAction.FINISHED,
    ]
    assert sleep.call_count == len(statuses)
    assert result.result.action_status == CurrentAction.FINISHED
    assert result.result.action_id == 'action-1'
    goal_handle.succeed.assert_called_once()


def test_cancel_order_cancels_active_nav2_goal(adapter):
    nav_handle = MagicMock()
    adapter._nav_to_pose_goal_handle = nav_handle
    goal_handle = _vda_goal_handle('cancelOrder')

    adapter.process_vda_action_callback(goal_handle)

    nav_handle.cancel_goal_async.assert_called_once()
    assert adapter._cancel_requested is True


def test_cancel_order_without_active_goal_sets_cancel_flag(adapter):
    goal_handle = _vda_goal_handle('cancelOrder')

    adapter.process_vda_action_callback(goal_handle)

    assert adapter._cancel_requested is True


# Cancellation


def test_cancel_callback_cancels_active_nav2_goal(adapter):
    nav_handle = MagicMock()
    adapter._nav_to_pose_goal_handle = nav_handle

    response = adapter.cancel_callback(MagicMock())

    assert response == CancelResponse.ACCEPT
    assert adapter._cancel_requested is True
    nav_handle.cancel_goal_async.assert_called_once()


def test_cancel_callback_without_active_goal_only_sets_flag(adapter):
    response = adapter.cancel_callback(MagicMock())

    assert response == CancelResponse.ACCEPT
    assert adapter._cancel_requested is True


def test_nav2_get_result_callback_clears_driving(adapter):
    adapter._driving = True
    future = MagicMock()

    adapter._nav_to_pose_amr_get_result_callback(future)

    assert adapter._driving is False


# Navigation


def test_navigate_to_node_sends_goal_and_succeeds(adapter):
    node = _vda_node()
    goal_handle = _nav_goal_handle(node)
    _setup_nav2_goal(adapter)

    result = asyncio.run(adapter.navigate_to_node_callback(goal_handle))

    assert isinstance(result, NavigateToNode.Result)
    nav_client = adapter._nav_to_pose_amr_action_client
    nav_client.wait_for_server.assert_called_once()
    goal = nav_client.send_goal_async.call_args.args[0]
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.header.stamp == Time(sec=10, nanosec=5)
    assert goal.pose.pose.position.x == pytest.approx(2.0)
    assert goal.pose.pose.position.y == pytest.approx(0.95)
    assert _yaw(goal.pose.pose.orientation) == pytest.approx(-0.66)

    goal_handle.succeed.assert_called_once()
    goal_handle.abort.assert_not_called()
    goal_handle.canceled.assert_not_called()
    assert adapter._active_node == node
    assert adapter._driving is False
    assert adapter._nav_to_pose_goal_handle is None


@pytest.mark.parametrize(
    'nav_status, expected_outcome',
    [
        (GoalStatus.STATUS_SUCCEEDED, 'succeed'),
        (GoalStatus.STATUS_CANCELED, 'canceled'),
        (GoalStatus.STATUS_ABORTED, 'abort'),
        (GoalStatus.STATUS_UNKNOWN, 'abort'),
    ],
)
def test_navigate_to_node_maps_nav2_result_to_goal_outcome(
    adapter, nav_status, expected_outcome
):
    goal_handle = _nav_goal_handle(_vda_node())
    _setup_nav2_goal(adapter, status=nav_status)

    asyncio.run(adapter.navigate_to_node_callback(goal_handle))

    getattr(goal_handle, expected_outcome).assert_called_once()
    for outcome in {'succeed', 'canceled', 'abort'} - {expected_outcome}:
        getattr(goal_handle, outcome).assert_not_called()


def test_navigate_to_node_aborts_when_nav2_rejects_goal(adapter):
    goal_handle = _nav_goal_handle(_vda_node())
    nav_handle = _setup_nav2_goal(adapter, accepted=False)

    result = asyncio.run(adapter.navigate_to_node_callback(goal_handle))

    assert isinstance(result, NavigateToNode.Result)
    goal_handle.abort.assert_called_once()
    goal_handle.succeed.assert_not_called()
    nav_handle.get_result_async.assert_not_called()
    assert adapter._driving is False
    assert adapter._nav_to_pose_goal_handle is None


def test_navigate_to_node_cancels_nav2_goal_when_cancel_requested_on_goal_handle(adapter):
    goal_handle = _nav_goal_handle(_vda_node(), is_cancel_requested=True)
    nav_handle = _setup_nav2_goal(adapter, status=GoalStatus.STATUS_CANCELED)

    asyncio.run(adapter.navigate_to_node_callback(goal_handle))

    nav_handle.cancel_goal_async.assert_called_once()
    goal_handle.canceled.assert_called_once()
    goal_handle.succeed.assert_not_called()


def test_navigate_to_node_forwards_cancel_received_before_goal_accepted(adapter):
    goal_handle = _nav_goal_handle(_vda_node())
    nav_handle = MagicMock(accepted=True)
    nav_handle.get_result_async = AsyncMock(
        return_value=MagicMock(status=GoalStatus.STATUS_CANCELED)
    )

    async def send_goal_while_cancel_arrives(_goal):
        adapter.cancel_callback(MagicMock())
        return nav_handle

    adapter._nav_to_pose_amr_action_client.send_goal_async = AsyncMock(
        side_effect=send_goal_while_cancel_arrives
    )

    asyncio.run(adapter.navigate_to_node_callback(goal_handle))

    nav_handle.cancel_goal_async.assert_called_once()
    goal_handle.canceled.assert_called_once()


def test_navigate_to_node_resets_state_when_sending_goal_fails(adapter):
    goal_handle = _nav_goal_handle(_vda_node())
    adapter._nav_to_pose_amr_action_client.send_goal_async = AsyncMock(
        side_effect=RuntimeError('nav2 unavailable')
    )

    with pytest.raises(RuntimeError, match='nav2 unavailable'):
        asyncio.run(adapter.navigate_to_node_callback(goal_handle))

    assert adapter._driving is False
    assert adapter._nav_to_pose_goal_handle is None


# Entry point


def test_main_spins_adapter_and_shuts_down(mocker):
    node_cls = mocker.patch.object(amr_adapter, 'AMRAdapter')
    executor_cls = mocker.patch.object(amr_adapter, 'MultiThreadedExecutor')
    init = mocker.patch.object(amr_adapter.rclpy, 'init')
    shutdown = mocker.patch.object(amr_adapter.rclpy, 'shutdown')

    amr_adapter.main(args=['--ros-args'])

    init.assert_called_once_with(args=['--ros-args'])
    executor = executor_cls.return_value
    executor.add_node.assert_called_once_with(node_cls.return_value)
    executor.spin.assert_called_once()
    node_cls.return_value.destroy_node.assert_called_once()
    shutdown.assert_called_once()
