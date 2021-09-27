/****************************************************************************
** Meta object code from reading C++ file 'controller.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../Controller/controller.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'controller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Controller_t {
    QByteArrayData data[24];
    char stringdata0[445];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Controller_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Controller_t qt_meta_stringdata_Controller = {
    {
QT_MOC_LITERAL(0, 0, 10), // "Controller"
QT_MOC_LITERAL(1, 11, 26), // "on_MVelSlider_valueChanged"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 5), // "value"
QT_MOC_LITERAL(4, 45, 27), // "on_MDistSlider_valueChanged"
QT_MOC_LITERAL(5, 73, 27), // "on_LVVelSlider_valueChanged"
QT_MOC_LITERAL(6, 101, 28), // "on_LVDistSlider_valueChanged"
QT_MOC_LITERAL(7, 130, 28), // "on_FV1VelSlider_valueChanged"
QT_MOC_LITERAL(8, 159, 29), // "on_FV1DistSlider_valueChanged"
QT_MOC_LITERAL(9, 189, 28), // "on_FV2VelSlider_valueChanged"
QT_MOC_LITERAL(10, 218, 29), // "on_FV2DistSlider_valueChanged"
QT_MOC_LITERAL(11, 248, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(12, 270, 11), // "UDPrecvDATA"
QT_MOC_LITERAL(13, 282, 17), // "UDPsock::UDP_DATA"
QT_MOC_LITERAL(14, 300, 11), // "display_Map"
QT_MOC_LITERAL(15, 312, 7), // "cv::Mat"
QT_MOC_LITERAL(16, 320, 18), // "on_LVBox_activated"
QT_MOC_LITERAL(17, 339, 5), // "index"
QT_MOC_LITERAL(18, 345, 19), // "on_FV1Box_activated"
QT_MOC_LITERAL(19, 365, 19), // "on_FV2Box_activated"
QT_MOC_LITERAL(20, 385, 15), // "on_Send_clicked"
QT_MOC_LITERAL(21, 401, 17), // "on_FV1_cf_toggled"
QT_MOC_LITERAL(22, 419, 7), // "checked"
QT_MOC_LITERAL(23, 427, 17) // "on_FV2_cf_toggled"

    },
    "Controller\0on_MVelSlider_valueChanged\0"
    "\0value\0on_MDistSlider_valueChanged\0"
    "on_LVVelSlider_valueChanged\0"
    "on_LVDistSlider_valueChanged\0"
    "on_FV1VelSlider_valueChanged\0"
    "on_FV1DistSlider_valueChanged\0"
    "on_FV2VelSlider_valueChanged\0"
    "on_FV2DistSlider_valueChanged\0"
    "on_pushButton_clicked\0UDPrecvDATA\0"
    "UDPsock::UDP_DATA\0display_Map\0cv::Mat\0"
    "on_LVBox_activated\0index\0on_FV1Box_activated\0"
    "on_FV2Box_activated\0on_Send_clicked\0"
    "on_FV1_cf_toggled\0checked\0on_FV2_cf_toggled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Controller[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   99,    2, 0x08 /* Private */,
       4,    1,  102,    2, 0x08 /* Private */,
       5,    1,  105,    2, 0x08 /* Private */,
       6,    1,  108,    2, 0x08 /* Private */,
       7,    1,  111,    2, 0x08 /* Private */,
       8,    1,  114,    2, 0x08 /* Private */,
       9,    1,  117,    2, 0x08 /* Private */,
      10,    1,  120,    2, 0x08 /* Private */,
      11,    0,  123,    2, 0x08 /* Private */,
      12,    1,  124,    2, 0x08 /* Private */,
      14,    1,  127,    2, 0x08 /* Private */,
      16,    1,  130,    2, 0x08 /* Private */,
      18,    1,  133,    2, 0x08 /* Private */,
      19,    1,  136,    2, 0x08 /* Private */,
      20,    0,  139,    2, 0x08 /* Private */,
      21,    1,  140,    2, 0x08 /* Private */,
      23,    1,  143,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 13,    3,
    0x80000000 | 15, 0x80000000 | 13,    3,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   22,
    QMetaType::Void, QMetaType::Bool,   22,

       0        // eod
};

void Controller::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Controller *_t = static_cast<Controller *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_MVelSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->on_MDistSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_LVVelSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->on_LVDistSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->on_FV1VelSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->on_FV1DistSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_FV2VelSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_FV2DistSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_pushButton_clicked(); break;
        case 9: _t->UDPrecvDATA((*reinterpret_cast< UDPsock::UDP_DATA(*)>(_a[1]))); break;
        case 10: { cv::Mat _r = _t->display_Map((*reinterpret_cast< UDPsock::UDP_DATA(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 11: _t->on_LVBox_activated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_FV1Box_activated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->on_FV2Box_activated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->on_Send_clicked(); break;
        case 15: _t->on_FV1_cf_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->on_FV2_cf_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject Controller::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_Controller.data,
      qt_meta_data_Controller,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Controller::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Controller::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Controller.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int Controller::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
