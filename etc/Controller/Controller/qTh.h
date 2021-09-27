#ifndef QTH_H
#define QTH_H

#include <QThread>
#include <sock_udp.hpp>

class qTh : public QThread
{
    Q_OBJECT
public:
    explicit qTh(QObject* parent = 0);
    int recvInit();
private:
    void run();
    UDPsock::UDPsocket UDPrecv;
signals:
    void setValue(UDPsock::UDP_DATA tmp);
};


#endif // QTH_H
