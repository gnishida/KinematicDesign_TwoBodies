/****************************************************************************
** Meta object code from reading C++ file 'OptionDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../OptionDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'OptionDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_OptionDialog_t {
    QByteArrayData data[8];
    char stringdata0[59];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OptionDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OptionDialog_t qt_meta_stringdata_OptionDialog = {
    {
QT_MOC_LITERAL(0, 0, 12), // "OptionDialog"
QT_MOC_LITERAL(1, 13, 7), // "onSmall"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 7), // "onLarge"
QT_MOC_LITERAL(4, 30, 6), // "onHalf"
QT_MOC_LITERAL(5, 37, 7), // "onTwice"
QT_MOC_LITERAL(6, 45, 4), // "onOK"
QT_MOC_LITERAL(7, 50, 8) // "onCancel"

    },
    "OptionDialog\0onSmall\0\0onLarge\0onHalf\0"
    "onTwice\0onOK\0onCancel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OptionDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x0a /* Public */,
       3,    0,   45,    2, 0x0a /* Public */,
       4,    0,   46,    2, 0x0a /* Public */,
       5,    0,   47,    2, 0x0a /* Public */,
       6,    0,   48,    2, 0x0a /* Public */,
       7,    0,   49,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void OptionDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        OptionDialog *_t = static_cast<OptionDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onSmall(); break;
        case 1: _t->onLarge(); break;
        case 2: _t->onHalf(); break;
        case 3: _t->onTwice(); break;
        case 4: _t->onOK(); break;
        case 5: _t->onCancel(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject OptionDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_OptionDialog.data,
      qt_meta_data_OptionDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *OptionDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OptionDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_OptionDialog.stringdata0))
        return static_cast<void*>(const_cast< OptionDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int OptionDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
