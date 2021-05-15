Import("env")
#
# Change build flags in runtime
#
env.ProcessUnFlags("-D ETHERCARD_DHCP")
env.Append(CPPDEFINES=("ETHERCARD_DHCP", 0))

env.ProcessUnFlags("-D ETHERCARD_TCPCLIENT")
env.Append(CPPDEFINES=("ETHERCARD_TCPCLIENT", 0))

env.ProcessUnFlags("-D ETHERCARD_TCPSERVER")
env.Append(CPPDEFINES=("ETHERCARD_TCPSERVER", 0))

env.ProcessUnFlags("-D ETHERCARD_ICMP")
env.Append(CPPDEFINES=("ETHERCARD_ICMP", 0))
