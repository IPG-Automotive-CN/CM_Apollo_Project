
#ifndef BRIDGE_TCP_CLIENT_H

#ifdef __cplusplus
    void tcp_send(std::string &data);
    uint32_t get_32b_little_endian(std::string &buffer, size_t offset);
    void append32bLittleEndian(std::string * buffer, uint32_t val);
#endif /* end test if C++ */

#endif /* BRIDGE_TCP_CLIENT_H */