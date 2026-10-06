import rclpy
from threading import Thread, Event

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
