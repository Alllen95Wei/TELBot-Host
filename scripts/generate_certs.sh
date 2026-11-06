#!/usr/bin/env bash
# ==============================================================================
# generate_certs.sh - Generates a Local Root CA and SAN Server Certificate
# for TELBot Host HTTPS + WebHID support over LAN / Wi-Fi / Tailscale.
# ==============================================================================
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CERT_DIR="${PROJECT_ROOT}/certs"
mkdir -p "${CERT_DIR}"

CA_KEY="${CERT_DIR}/ca.key"
CA_CRT="${CERT_DIR}/ca.crt"
SERVER_KEY="${CERT_DIR}/server.key"
SERVER_CSR="${CERT_DIR}/server.csr"
SERVER_CRT="${CERT_DIR}/server.crt"
OPENSSL_CNF="${CERT_DIR}/openssl-san.cnf"

HOSTNAME_SHORT="$(hostname)"
HOSTNAME_LOCAL="${HOSTNAME_SHORT}.local"

# Build Subject Alternative Name (SAN) list dynamically from all host IPs + standard local addresses
declare -A SEEN_IPS=()
SAN_ENTRIES=(
    "DNS:localhost"
    "DNS:${HOSTNAME_SHORT}"
    "DNS:${HOSTNAME_LOCAL}"
    "IP:127.0.0.1"
    "IP:::1"
)
SEEN_IPS["127.0.0.1"]=1
SEEN_IPS["::1"]=1

for ip in $(hostname -I 2>/dev/null || true) 192.168.0.50 100.111.106.89 192.168.67.1; do
    if [[ -n "${ip}" && -z "${SEEN_IPS[${ip}]:-}" ]]; then
        SAN_ENTRIES+=("IP:${ip}")
        SEEN_IPS["${ip}"]=1
    fi
done

SAN_STRING="$(IFS=,; echo "${SAN_ENTRIES[*]}")"

cat > "${OPENSSL_CNF}" <<EOF
[req]
default_bits       = 2048
prompt             = no
default_md         = sha256
distinguished_name = dn
req_extensions     = v3_req

[dn]
C  = TW
O  = TELBot Robotics
CN = ${HOSTNAME_SHORT}

[v3_ca]
subjectKeyIdentifier   = hash
authorityKeyIdentifier = keyid:always,issuer
basicConstraints       = critical, CA:TRUE
keyUsage               = critical, digitalSignature, cRLSign, keyCertSign

[v3_req]
basicConstraints = critical, CA:FALSE
keyUsage         = critical, digitalSignature, keyEncipherment
extendedKeyUsage = serverAuth
subjectAltName   = ${SAN_STRING}
EOF

if [[ ! -f "${CA_KEY}" || ! -f "${CA_CRT}" ]]; then
    echo "[*] Generating TELBot Local Root CA (${CA_CRT})..."
    openssl genrsa -out "${CA_KEY}" 2048 2>/dev/null
    openssl req -x509 -new -nodes -key "${CA_KEY}" -sha256 -days 3650 \
        -subj "/C=TW/O=TELBot Robotics/CN=TELBot Local Root CA" \
        -config "${OPENSSL_CNF}" -extensions v3_ca \
        -out "${CA_CRT}"
else
    echo "[*] Reusing existing TELBot Local Root CA (${CA_CRT})."
fi

echo "[*] Generating Server Certificate (${SERVER_CRT}) with SANs:"
echo "    ${SAN_STRING}"
openssl genrsa -out "${SERVER_KEY}" 2048 2>/dev/null
openssl req -new -key "${SERVER_KEY}" -config "${OPENSSL_CNF}" -out "${SERVER_CSR}"
openssl x509 -req -in "${SERVER_CSR}" \
    -CA "${CA_CRT}" -CAkey "${CA_KEY}" -CAcreateserial \
    -out "${SERVER_CRT}" -days 825 -sha256 \
    -extfile "${OPENSSL_CNF}" -extensions v3_req 2>/dev/null

rm -f "${SERVER_CSR}" "${CERT_DIR}/ca.srl"
chmod 600 "${CA_KEY}" "${SERVER_KEY}"
chmod 644 "${CA_CRT}" "${SERVER_CRT}"

echo "[+] Certificates generated successfully in ${CERT_DIR}:"
echo "    - Root CA (install on client to remove browser warnings): ${CA_CRT}"
echo "    - Server Certificate: ${SERVER_CRT}"
echo "    - Server Private Key: ${SERVER_KEY}"

