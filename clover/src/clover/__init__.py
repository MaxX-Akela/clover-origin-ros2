import atexit
import os
import time
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from threading import Thread, Event, Lock

def long_callback(fn):
    """
    Decorator for long-running topic callbacks, primarily for image processing.
    The callback is run in a separate thread, so it doesn't block the executor,
    and only the latest message is processed.

    Usage example:

    @long_callback
    def image_callback(msg):
        # perform image processing
        # ...

    node.create_subscription(Image, 'main_camera/image_raw', image_callback, 1)
    """
    e = Event()

    def thread():
        while True:
            e.wait()
            e.clear()
            if not rclpy.ok():
                break
            fn(thread.current_msg)

    thread.current_msg = None
    Thread(target=thread, daemon=True).start()

    def wrapper(msg):
        thread.current_msg = msg
        e.set()

    return wrapper


_proxy_lock = Lock()
_proxy_node = None

def _get_proxy_node():
    # the node of the service proxies, created on the first use and spun in a background thread
    global _proxy_node
    with _proxy_lock:
        if _proxy_node is None:
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node('clover_service_proxy_%d' % os.getpid(), start_parameter_services=False)
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            def spin():
                try:
                    executor.spin()
                except (KeyboardInterrupt, ExternalShutdownException):
                    pass

            thread = Thread(target=spin, daemon=True)
            thread.start()

            def shutdown():
                # stop spinning before the interpreter exits
                executor.shutdown(timeout_sec=1.0)
                rclpy.try_shutdown()
                thread.join(1.0)

            atexit.register(shutdown)
            _proxy_node = node
        return _proxy_node


def service_proxy(name, srv_type, timeout=None, wait_for_service=5.0):
    """
    Create a function, that calls a ROS service synchronously and returns the response,
    similar to rospy.ServiceProxy in ROS 1. The request fields are passed as arguments,
    the omitted fields have the default values.

    The function may be called from any thread, including the callbacks of your own node
    (the proxies use a separate node, which is spun in a background thread). rclpy is
    initialized automatically, if it's not initialized yet.

    name: name of the service
    srv_type: type of the service
    timeout: time to wait for the response, in seconds; None is to wait forever
    wait_for_service: time to wait for the service to become available, in seconds

    The function raises RuntimeError if the service is not available and TimeoutError
    if there is no response in timeout.

    Usage example:

    from clover import srv, service_proxy

    navigate = service_proxy('navigate', srv.Navigate)
    get_telemetry = service_proxy('get_telemetry', srv.GetTelemetry)

    navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    print(get_telemetry(frame_id='map').x)
    """
    node = _get_proxy_node()
    with _proxy_lock:
        client = node.create_client(srv_type, name)
    full_name = node.resolve_service_name(name)

    def proxy(*args, **kwargs):
        request = srv_type.Request()
        fields = request.get_fields_and_field_types()
        if len(args) > len(fields):
            raise TypeError('%s takes at most %d arguments (%d given)' % (full_name, len(fields), len(args)))
        values = dict(zip(fields, args))
        for key, value in kwargs.items():
            if key not in fields:
                raise TypeError('%s got an unexpected argument \'%s\', the request fields are: %s' %
                                (full_name, key, ', '.join(fields)))
            if key in values:
                raise TypeError('%s got multiple values for argument \'%s\'' % (full_name, key))
            values[key] = value
        for key, value in values.items():
            if fields[key] in ('float', 'double') and isinstance(value, int):
                value = float(value) # integers are accepted for the float fields, as in ROS 1
            setattr(request, key, value)

        if not client.wait_for_service(timeout_sec=wait_for_service):
            raise RuntimeError('service [%s] unavailable' % full_name)

        done = Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        deadline = None if timeout is None else time.monotonic() + timeout
        while not done.wait(0.1):
            if not rclpy.ok():
                raise RuntimeError('service [%s] call is interrupted by shutdown' % full_name)
            if deadline is not None and time.monotonic() > deadline:
                future.cancel()
                raise TimeoutError('service [%s] did not respond in %g s' % (full_name, timeout))
        return future.result()

    return proxy
