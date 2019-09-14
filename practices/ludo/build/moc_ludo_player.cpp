/****************************************************************************
** Meta object code from reading C++ file 'ludo_player.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.2.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ludo/ludo_player.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ludo_player.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.2.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ludo_player_t {
    QByteArrayData data[10];
    char stringdata[129];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_ludo_player_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_ludo_player_t qt_meta_stringdata_ludo_player = {
    {
QT_MOC_LITERAL(0, 0, 11),
QT_MOC_LITERAL(1, 12, 12),
QT_MOC_LITERAL(2, 25, 0),
QT_MOC_LITERAL(3, 26, 13),
QT_MOC_LITERAL(4, 40, 10),
QT_MOC_LITERAL(5, 51, 18),
QT_MOC_LITERAL(6, 70, 8),
QT_MOC_LITERAL(7, 79, 18),
QT_MOC_LITERAL(8, 98, 16),
QT_MOC_LITERAL(9, 115, 12)
    },
    "ludo_player\0select_piece\0\0turn_complete\0"
    "start_turn\0positions_and_dice\0relative\0"
    "post_game_analysis\0std::vector<int>\0"
    "relative_pos\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ludo_player[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06,
       3,    1,   37,    2, 0x06,

 // slots: name, argc, parameters, tag, flags
       4,    1,   40,    2, 0x0a,
       7,    1,   43,    2, 0x0a,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Bool,    2,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 8,    9,

       0        // eod
};

void ludo_player::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ludo_player *_t = static_cast<ludo_player *>(_o);
        switch (_id) {
        case 0: _t->select_piece((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->turn_complete((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->start_turn((*reinterpret_cast< positions_and_dice(*)>(_a[1]))); break;
        case 3: _t->post_game_analysis((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ludo_player::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ludo_player::select_piece)) {
                *result = 0;
            }
        }
        {
            typedef void (ludo_player::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ludo_player::turn_complete)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject ludo_player::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_ludo_player.data,
      qt_meta_data_ludo_player,  qt_static_metacall, 0, 0}
};


const QMetaObject *ludo_player::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ludo_player::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ludo_player.stringdata))
        return static_cast<void*>(const_cast< ludo_player*>(this));
    return QObject::qt_metacast(_clname);
}

int ludo_player::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void ludo_player::select_piece(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ludo_player::turn_complete(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
