INTF="${1:-enP8p1s0}"
IP_ADDR=$(ip -o -4 addr show dev enP8p1s0 | awk '{print $4}' | cut -d'/' -f1)
LAST_OCTET=$(echo "$IP_ADDR" | cut -d'.' -f4)
MAC_PREFIX=$(cat /sys/class/net/$INTF/address 2>/dev/null | cut -d':' -f1-5)

IDS=("2" "9")

for ID in "${IDS[@]}"; do
  sudo nmcli connection add type vlan con-name $INTF.$ID dev $INTF id $ID
  sudo nmcli connection modify $INTF.$ID ipv4.addresses 192.168.$ID.$LAST_OCTET/24 ipv4.method manual ipv4.routing-rules "priority 1000 from 192.168.$ID.$LAST_OCTET table $ID" ipv4.routes "192.168.$ID.0/24 table=$ID" 802-3-ethernet.cloned-mac-address $MAC_PREFIX:$ID
  sudo nmcli connection up $INTF.$ID
done

