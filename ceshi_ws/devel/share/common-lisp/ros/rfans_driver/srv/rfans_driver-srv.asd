
(cl:in-package :asdf)

(defsystem "rfans_driver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RfansCommand" :depends-on ("_package_RfansCommand"))
    (:file "_package_RfansCommand" :depends-on ("_package"))
    (:file "null" :depends-on ("_package_null"))
    (:file "_package_null" :depends-on ("_package"))
  ))