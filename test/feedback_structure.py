import sys

# 먼저 SDK examples 경로 추가
sys.path.append("/home/icatheon/Kinova-kortex2_Gen3_G3L/api_python/examples")

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2
import utilities

# ============ 재귀적으로 객체 구조 출력 ============
def print_object(obj, indent=0):
    prefix = " " * indent
    if hasattr(obj, "__dict__"):
        print(f"{prefix}{type(obj).__name__}:")
        for k, v in vars(obj).items():
            print(f"{prefix}  {k} -> {type(v).__name__}")
            print_object(v, indent + 4)
    elif isinstance(obj, list):
        for i, item in enumerate(obj):
            print(f"{prefix}[{i}] -> {type(item).__name__}")
            print_object(item, indent + 4)
    else:
        print(f"{prefix}{obj} ({type(obj).__name__})")

# ============ main ============
def main():
    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # feedback 가져오기
        feedback = base_cyclic.RefreshFeedback()

        print("===== Feedback 객체 전체 구조 =====")
        print_object(feedback)

if __name__ == "__main__":
    main()
