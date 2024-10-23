from time import perf_counter

from tpc.tests.communication.test_ros2_communication_handles import( 
    TestROSCommunicationHandlers,
)

test = TestROSCommunicationHandlers()
# test.test_ros_server_client_naming()

# start = perf_counter()
# test.test_ros_server_client_communication_executor()
# print(f"Time taken: {perf_counter() - start}")

# start = perf_counter()
# test.test_ros_server_client_communication_threading()
# print(f"Time taken: {perf_counter() - start}")

start = perf_counter()
test.test_ros_server_client_communication_spin_once()
print(f"Time taken: {perf_counter() - start}")
