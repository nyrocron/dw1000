#!/usr/bin/env python3

def bit(n):
    return 1 << n


flags = [
    (bit(0), 'IRQS'),
    (bit(1), 'CPLOCK'),
    (bit(2), 'ESYNCR'),
    (bit(3), 'AAT'),
    (bit(4), 'TXFRB'),
    (bit(5), 'TXPRS'),
    (bit(6), 'TXPHS'),
    (bit(7), 'TXFRS'),
    (bit(8), 'RXPRD'),
    (bit(9), 'RXSFDD'),
    (bit(10), 'LDEDONE'),
    (bit(11), 'RXPHD'),
    (bit(12), 'RXPHE'),
    (bit(13), 'RXDFR'),
    (bit(14), 'RXFCG'),
    (bit(15), 'RXFCE'),
    (bit(16), 'RXRFSL'),
    (bit(17), 'RXRFTO'),
    (bit(18), 'LDEERR'),
    (bit(20), 'RXOVRR'),
    (bit(21), 'RXPTO'),
    (bit(22), 'GPIOIRQ'),
    (bit(23), 'SLP2INIT'),
    (bit(24), 'RFPLL_LL'),
    (bit(25), 'CLKPLL_LL'),
    (bit(26), 'RXSFDTO'),
    (bit(27), 'HPDWARN'),
    (bit(28), 'TXBERR'),
    (bit(29), 'AFFREJ'),
    (bit(30), 'HSRBP'),
    (bit(31), 'ICRBP'),
]


def read_flags(val):
    active_flags = []
    for flag, name in flags:
        if val & flag:
            active_flags.append(name)
    return active_flags


if __name__ == '__main__':
    from sys import argv

    value = int(argv[1], 16)
    for flag_name in read_flags(value):
        print(flag_name)

